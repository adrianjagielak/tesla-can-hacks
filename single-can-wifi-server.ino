/*
   ESP32 + SN65HVD230
   Wi-Fi TCP server for tesLAX iOS app
   Listens on TCP port 35000
   Emulates ELM327 + ST commands
*/

#include <WiFi.h>
#include "driver/twai.h"

// ---------- Wi-Fi Credentials (DHCP) ----------
#define WIFI_SSID "iPhone 16 Pro (Adrian)"
#define WIFI_PASS "12345678"

// ---------- TCP Port for tesLAX ----------
#define TESLAX_TCP_PORT 35000

// ---------- CAN Pins (ESP32 hardware) ----------
#define RX_PIN 16
#define TX_PIN 17

// ---------- Buffers and Config ----------
#define MAX_CMD_LEN 128
#define STREAM_INTERVAL_MS 5       // Delay between sending CAN frames
#define PROMPT_STR "\r>"

WiFiServer server(TESLAX_TCP_PORT);
WiFiClient client;  // single client approach

// For storing partial command input from TCP
char cmdBuffer[MAX_CMD_LEN];
int cmdPos = 0;

bool deviceConnected = false;

// "ids" array: if true => pass that ID
bool ids[2048];
bool noFilter = true;

// Flag to indicate if we should actually stream CAN data
bool streamingEnabled = false;

static char sendBuf[2048];
static size_t sendPos = 0;

//////////////////////////////////////////////////////
// Setup CAN/TWAI
//////////////////////////////////////////////////////
bool setupCAN() {
  // General config
  twai_general_config_t g_config = {
    .mode = TWAI_MODE_LISTEN_ONLY,   // or TWAI_MODE_NORMAL
    .tx_io = (gpio_num_t)TX_PIN,
    .rx_io = (gpio_num_t)RX_PIN,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 1,
    .rx_queue_len = 256,
    .alerts_enabled = TWAI_ALERT_ALL,
    .clkout_divider = 0
  };

  // 500 kbps
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    return false;
  }

  // Only watch for RX data, RX queue full
  twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL, NULL);

  // Default: accept all IDs
  memset(ids, 1, sizeof(ids));
  return true;
}

//////////////////////////////////////////////////////
// Helper: send strings to the TCP client
//////////////////////////////////////////////////////
void sendToClient(const char* data) {
  if (deviceConnected && client && client.connected()) {
    client.print(data);
  }
}

//////////////////////////////////////////////////////
// Helper: ELM prompt
//////////////////////////////////////////////////////
void sendPrompt() {
  sendToClient(PROMPT_STR);
}

//////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////
void processCommand(const char* cmd);
void handleElmCommand(const char* cmd);
void startStreaming();
void stopStreaming();

//////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.println("Starting TESLAX Wi-Fi adapter...");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to iPhone hotspot: %s ...\n", WIFI_SSID);

  // Wait up to ~10s for connect
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to Wi-Fi. Continuing anyway...");
  }

  // Start TCP server
  server.begin();
  server.setNoDelay(true); // For lower latency

  // Setup CAN
  if (!setupCAN()) {
    Serial.println("Error initializing CAN. Check pins/driver.");
  }
}

//////////////////////////////////////////////////////
// Main Loop
//////////////////////////////////////////////////////
unsigned long lastStreamTime = 0;

void loop() {
  // Check if we have a connected client or a new one
  if (!client || !client.connected()) {
    // If our old client is gone, free it up
    if (client) {
      client.stop();
      streamingEnabled = false;
    }
    // Accept new client if available
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      client.setNoDelay(true);
      deviceConnected = true;
      Serial.println("TCP client connected.");
      // Send an ELM banner + prompt
      // Many OBD apps expect exactly something like this upon connect
      sendToClient("ELM327 v1.5\r\n");
      sendToClient("\r\n>");
    } else {
      deviceConnected = false;
    }
  }

  // If connected, handle input & streaming
  if (deviceConnected) {
    // 1) Read any incoming data -> parse commands
    while (client.available()) {
      char c = client.read();
      if (c == '\r' || c == '\n') {
        if (cmdPos > 0) {
          cmdBuffer[cmdPos] = 0;
          processCommand(cmdBuffer);
          cmdPos = 0;
        }
      } else {
        if (cmdPos < (int)sizeof(cmdBuffer) - 1) {
          cmdBuffer[cmdPos++] = tolower((unsigned char)c);
        }
      }
    }

    // 2) If streaming is enabled, send out any CAN frames
    if (streamingEnabled && (millis() - lastStreamTime >= STREAM_INTERVAL_MS)) {
        twai_message_t msg;
        uint32_t alerts;
    
        // Read any alerts non-blocking
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
        
            // Handle newly arrived data
            if (alerts & TWAI_ALERT_RX_DATA) {
                int batchSize = 0;
                while (twai_receive(&msg, 0) == ESP_OK && batchSize < 64) {
                    if (!noFilter && !ids[msg.identifier]) {
                        // Skip this ID
                    } else {
                        // Build a single-frame string into outBuf
                        // e.g. "12311223344\r"
                        char outBuf[50];
                        int len = snprintf(outBuf, sizeof(outBuf), "%03X", msg.identifier);
                        for (int i = 0; i < msg.data_length_code; i++) {
                            len += snprintf(&outBuf[len], sizeof(outBuf) - len, "%02X", msg.data[i]);
                        }
                        outBuf[len++] = '\r';
                        outBuf[len]   = '\0';
                    
                        // Now add it to our larger sendBuf in one pass
                        size_t needed = strlen(outBuf); // bytes needed for this frame
                        size_t remaining = sizeof(sendBuf) - sendPos;
                    
                        // If there's not enough space left, flush what we have so far
                        if (needed > remaining) {
                            sendBuf[sendPos] = '\0'; 
                            sendToClient(sendBuf);  // 1 big TCP write
                            sendPos = 0;
                        }
                    
                        // Copy outBuf into sendBuf
                        memcpy(sendBuf + sendPos, outBuf, needed);
                        sendPos += needed;
                    }
                    batchSize++;
                }
            }
        
            // If queue was full, drain it similarly
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                while (twai_receive(&msg, 0) == ESP_OK) {
                    if (!noFilter && !ids[msg.identifier]) {
                        continue;
                    }
                    // Build frame string
                    char outBuf[50];
                    int len = snprintf(outBuf, sizeof(outBuf), "%03X", msg.identifier);
                    for (int i = 0; i < msg.data_length_code; i++) {
                        len += snprintf(&outBuf[len], sizeof(outBuf) - len, "%02X", msg.data[i]);
                    }
                    outBuf[len++] = '\r';
                    outBuf[len]   = '\0';
                
                    // Add to chunk buffer
                    size_t needed = strlen(outBuf);
                    size_t remaining = sizeof(sendBuf) - sendPos;
                    if (needed > remaining) {
                        sendBuf[sendPos] = '\0';
                        sendToClient(sendBuf);
                        sendPos = 0;
                    }
                    memcpy(sendBuf + sendPos, outBuf, needed);
                    sendPos += needed;
                }
            }
        }
    
        // After processing both RX_DATA and RX_QUEUE_FULL, flush anything in sendBuf:
        if (sendPos > 0) {
            sendBuf[sendPos] = '\0';  // Terminate
            sendToClient(sendBuf);    // 1 big TCP write
            sendPos = 0;
        }
    
        lastStreamTime = millis();
    }
  }
}

//////////////////////////////////////////////////////
// Process an incoming command line
//////////////////////////////////////////////////////
void processCommand(const char* cmd) {
  // Example unrecognized can be a single space or empty
  if (strlen(cmd) == 0) {
    // just a blank line
    sendToClient("OK\r>");
    return;
  }
  if (strcmp(cmd, " ") == 0) {
    // space
    sendToClient("OK\r>");
    return;
  }

  // TeslaX might send ST commands or ELM commands
  if (strncmp(cmd, "st", 2) == 0) {
    // ST command
    // parse out which one
    if (strncmp(cmd, "stfap ", 6) == 0) {
      const char* arg = cmd + 6;
      char* comma = strchr((char*)arg, ',');
      if (comma) {
        *comma = 0;
        uint16_t filterId = strtol(arg, NULL, 16);
        if (noFilter) {
          memset(ids, 0, sizeof(ids));
          noFilter = false;
        }
        ids[filterId] = true;
      }
      sendToClient("OK\r>");
    }
    else if (strncmp(cmd, "stfcp", 5) == 0) {
      // "stfcp" => clear all filters => pass all
      memset(ids, 1, sizeof(ids));
      noFilter = true;
      sendToClient("OK\r>");
    }
    else if (strcmp(cmd, "stm") == 0) {
      // "stm" => start monitoring
      // We'll enable streaming
      streamingEnabled = true;
      sendToClient("OK\r>");
    }
    else if (strcmp(cmd, "stdi") == 0) {
      // ST device info
      // Return some fake string
      sendToClient("STN2100 v4.3.1\r>");
    }
    else if (strncmp(cmd, "stp", 3) == 0) {
      // e.g. "stp33" => set protocol 0x33
      sendToClient("OK\r>");
    }
    else {
      // Possibly more ST commands
      Serial.printf("Unrecognized ST cmd: %s\n", cmd);
      sendToClient("OK\r>");
    }
  }
  else if (strncmp(cmd, "at", 2) == 0) {
    // ELM327-like command
    handleElmCommand(cmd);
  }
  else {
    // Catch all
    Serial.printf("Unrecognized cmd: %s\n", cmd);
    sendToClient("OK\r>");
  }
}

//////////////////////////////////////////////////////
// Handle some typical ELM commands
//////////////////////////////////////////////////////
void handleElmCommand(const char* cmd) {
  if (strcmp(cmd, "ati") == 0) {
    sendToClient("ELM327 v1.5\r>");
  }
  else if (strcmp(cmd, "at@1") == 0) {
    sendToClient("tesLAX WiFi Adapter\r>");
  }
  else if (strcmp(cmd, "atz") == 0) {
    // Reset
    sendToClient("ELM327 v1.5\r\r>");
  }
  else if (strcmp(cmd, "ate0") == 0) {
    // echo off
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "ate1") == 0) {
    // echo on
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "ath0") == 0) {
    // headers off
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "ath1") == 0) {
    // headers on
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "atsp0") == 0) {
    // auto protocol
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "atrv") == 0) {
    // read voltage
    sendToClient("12.6V\r>");
  }
  else if (strcmp(cmd, "ats0") == 0) {
    // "Spaces Off" command
    sendToClient("OK\r>");
  }
  else if (strcmp(cmd, "atcaf0") == 0) {
    // CAN auto formatting OFF
    sendToClient("OK\r>");
  }
  else {
    Serial.printf("Unrecognized AT cmd: %s\n", cmd);
    sendToClient("OK\r>");
  }
}
