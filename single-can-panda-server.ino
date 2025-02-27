/*
   ESP32 + SN65HVD230
   UDP server implementing Panda v2 protocol for tesLAX iOS app
   Listens on UDP port 1338
*/

#include <WiFi.h>
#include <AsyncUDP.h>
#include "driver/twai.h"

// ---------- Wi-Fi Credentials (DHCP) ----------
#define WIFI_SSID "iPhone 16 Pro (Adrian)"
#define WIFI_PASS "12345678"

// ---------- Panda UDP Port ----------
#define PANDA_UDP_PORT 1338

// ---------- CAN Pins (ESP32 hardware) ----------
#define RX_PIN 16
#define TX_PIN 17

// ---------- Protocol Constants ----------
#define PANDA_TIMEOUT_MS 10000  // 10 seconds timeout (2x the required 5s heartbeat)
#define SI_CHAR 0x0F   // Shift In - Add filters
#define SO_CHAR 0x0E   // Shift Out - Remove filters
#define FF_CHAR 0x0C   // Form Feed - Send all frames
#define CAN_CHAR 0x18  // Cancel - Clear all filters

#define MAX_FRAMES_PER_PACKET 92  // (1472 bytes / 16 bytes per frame)
#define SEND_INTERVAL_MS 50       // Send the buffer at least every 50ms

AsyncUDP udp;
IPAddress clientIP;
uint16_t clientPort = 0;
unsigned long lastHeartbeat = 0;
bool streamingEnabled = false;

// Filter management
struct FilterEntry {
    uint8_t busId;
    uint16_t frameId;
    bool active;
};
#define MAX_FILTERS 256
FilterEntry filters[MAX_FILTERS];
int numFilters = 0;
bool sendAllFrames = false;

// Panda packet structure (16 bytes)
struct __attribute__((packed)) PandaPacket {
    uint32_t f1;  // Frame ID shifted left 21 bits
    uint32_t f2;  // Bus ID << 4 | Length
    uint8_t data[8];  // CAN data
};

// Buffer for collecting frames
PandaPacket frameBuffer[MAX_FRAMES_PER_PACKET];
int bufferIndex = 0;

unsigned long lastSend = 0;

//////////////////////////////////////////////////////
// Setup CAN/TWAI
//////////////////////////////////////////////////////
bool setupCAN() {
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_LISTEN_ONLY,
        .tx_io = (gpio_num_t)TX_PIN,
        .rx_io = (gpio_num_t)RX_PIN,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 1,
        .rx_queue_len = 256,
        .alerts_enabled = TWAI_ALERT_ALL,
        .clkout_divider = 0
    };

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

    twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL, NULL);
    return true;
}

//////////////////////////////////////////////////////
// Filter Management
//////////////////////////////////////////////////////
void clearAllFilters() {
    numFilters = 0;
    sendAllFrames = false;
}

bool addFilter(uint8_t busId, uint16_t frameId) {
    if (numFilters >= MAX_FILTERS) return false;
    
    // Check if filter already exists
    for (int i = 0; i < numFilters; i++) {
        if (filters[i].busId == busId && filters[i].frameId == frameId) {
            filters[i].active = true;
            return true;
        }
    }
    
    // Add new filter
    filters[numFilters].busId = busId;
    filters[numFilters].frameId = frameId;
    filters[numFilters].active = true;
    numFilters++;
    return true;
}

bool removeFilter(uint8_t busId, uint16_t frameId) {
    for (int i = 0; i < numFilters; i++) {
        if (filters[i].busId == busId && filters[i].frameId == frameId) {
            filters[i].active = false;
            return true;
        }
    }
    return false;
}

bool shouldPassFrame(uint8_t busId, uint16_t frameId) {
    if (sendAllFrames) return true;
    
    for (int i = 0; i < numFilters; i++) {
        if (filters[i].active) {
            if ((filters[i].busId == 0xFF || filters[i].busId == busId) &&
                filters[i].frameId == frameId) {
                return true;
            }
        }
    }
    return false;
}

// Helper to safely add frame to buffer & send if needed
void addFrame(uint32_t id, uint8_t* data, uint8_t len) {
    // Check if adding this frame would overflow
    if (bufferIndex >= MAX_FRAMES_PER_PACKET) {
        // Buffer full - send it now
        size_t sendSize = bufferIndex * sizeof(PandaPacket);
        udp.writeTo((uint8_t*)frameBuffer, sendSize, clientIP, clientPort);
        bufferIndex = 0;
    }
    
    // Add frame to buffer
    frameBuffer[bufferIndex].f1 = id << 21;
    frameBuffer[bufferIndex].f2 = len | (0 << 4);
    memcpy(frameBuffer[bufferIndex].data, data, len);
    bufferIndex++;
}

//////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    Serial.println("Starting Panda UDP adapter...");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // Setup UDP
    if(udp.listen(PANDA_UDP_PORT)) {
        Serial.print("UDP Listening on port ");
        Serial.println(PANDA_UDP_PORT);
        
        udp.onPacket([](AsyncUDPPacket packet) {
            // Handle incoming UDP packets
            if (packet.length() == 5 && memcmp(packet.data(), "ehllo", 5) == 0) {
                // v2 Protocol initiation/heartbeat
                clientIP = packet.remoteIP();
                clientPort = packet.remotePort();
                lastHeartbeat = millis();
                streamingEnabled = true;
                
                // Send ACK frame (bus 15, id 0x06)
                PandaPacket ack = {0};
                ack.f1 = 0x06 << 21;
                ack.f2 = (15 << 4) | 8;  // bus 15, length 8
                udp.writeTo((uint8_t*)&ack, sizeof(PandaPacket), clientIP, clientPort);
                
                Serial.println("Client connected with v2 protocol");
            }
            else if (packet.length() > 0) {
                // Handle filter commands
                uint8_t cmd = packet.data()[0];
                if (cmd == SI_CHAR || cmd == SO_CHAR) {
                    // Add/remove filters
                    int numPairs = (packet.length() - 1) / 3;
                    for (int i = 0; i < numPairs; i++) {
                        uint8_t busId = packet.data()[1 + (i * 3)];
                        uint16_t frameId = (packet.data()[2 + (i * 3)] << 8) | packet.data()[3 + (i * 3)];
                        if (cmd == SI_CHAR) {
                            addFilter(busId, frameId);
                        } else {
                            removeFilter(busId, frameId);
                        }
                    }
                }
                else if (cmd == CAN_CHAR) {
                    clearAllFilters();
                }
                else if (cmd == FF_CHAR) {
                    sendAllFrames = true;
                }
            }
        });
    }

    // Setup CAN
    if (!setupCAN()) {
        Serial.println("Error initializing CAN. Check pins/driver.");
    }
}

//////////////////////////////////////////////////////
// Main Loop
//////////////////////////////////////////////////////
void loop() {
    // Check for client timeout
    if (streamingEnabled && (millis() - lastHeartbeat > PANDA_TIMEOUT_MS)) {
        Serial.println("Client timeout - stopping stream");
        streamingEnabled = false;
        clientPort = 0;
        clearAllFilters();
    }

    // Process CAN messages if we have an active client
    if (streamingEnabled) {
        uint32_t alerts;
        twai_message_t msg;

        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_RX_DATA) {
                while (twai_receive(&msg, 0) == ESP_OK) {
                    if (shouldPassFrame(0, msg.identifier)) {
                        addFrame(msg.identifier, msg.data, msg.data_length_code);
                    }
                }
            }
        }

        // Send buffer if enough time has passed
        if (bufferIndex > 0 && millis() - lastSend >= SEND_INTERVAL_MS) {
            size_t sendSize = bufferIndex * sizeof(PandaPacket);
            udp.writeTo((uint8_t*)frameBuffer, sendSize, clientIP, clientPort);
            bufferIndex = 0;
            lastSend = millis();
        }
    }
}
