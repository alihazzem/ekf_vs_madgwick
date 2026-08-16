#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- BLE UUIDs (Must match the RX ESP32 exactly) ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- Telemetry Packet ---
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;           // 0xAA
    uint8_t header2;           // 0x55
    int16_t thigh_l;
    int16_t shin_l;
    int16_t knee_l;
    int16_t thigh_r;
    int16_t shin_r;
    int16_t knee_r;
    uint8_t status;
    uint8_t checksum;
} EspTelemetry_t;
#pragma pack(pop)

uint8_t pktBuffer[sizeof(EspTelemetry_t)];
int pktIndex = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      deviceConnected = true;
      Serial.println("Device connected");
      // Force aggressive 7.5ms connection interval (6 * 1.25ms = 7.5ms)
      esp_ble_conn_update_params_t conn_params = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      conn_params.min_int = 6;
      conn_params.max_int = 6;
      conn_params.latency = 0;
      conn_params.timeout = 100;
      esp_ble_gap_update_conn_params(&conn_params);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

void setup() {
  Serial.begin(115200);
  
  // UART2 from STM32 (default RX is GPIO 16, TX is GPIO 17)
  // Connect STM32 PA11 (TX) to ESP32 GPIO 16 (RX2)
  Serial2.begin(115200); 
  
  Serial.println("Starting ESP32 Leg Transmitter (Server)...");

  // Create the BLE Device
  BLEDevice::init("ESP32_LEG_TX");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
    // Read from STM32 over UART2
    while (Serial2.available() > 0) {
        uint8_t b = Serial2.read();
        
        // State machine to parse incoming packet
        if (pktIndex == 0) {
            if (b == 0xAA) {
                pktBuffer[pktIndex++] = b;
            }
        } else if (pktIndex == 1) {
            if (b == 0x55) {
                pktBuffer[pktIndex++] = b;
            } else {
                pktIndex = 0; // Lost sync
            }
        } else if (pktIndex < sizeof(EspTelemetry_t)) {
            pktBuffer[pktIndex++] = b;
            
            if (pktIndex == sizeof(EspTelemetry_t)) {
                // We have a full packet. Check checksum!
                EspTelemetry_t* rxPkt = (EspTelemetry_t*)pktBuffer;
                
                uint8_t chk = rxPkt->header1 + rxPkt->header2;
                chk += (uint8_t)(rxPkt->thigh_l & 0xFF) + (uint8_t)(rxPkt->thigh_l >> 8);
                chk += (uint8_t)(rxPkt->shin_l  & 0xFF) + (uint8_t)(rxPkt->shin_l  >> 8);
                chk += (uint8_t)(rxPkt->knee_l  & 0xFF) + (uint8_t)(rxPkt->knee_l  >> 8);
                chk += (uint8_t)(rxPkt->thigh_r & 0xFF) + (uint8_t)(rxPkt->thigh_r >> 8);
                chk += (uint8_t)(rxPkt->shin_r  & 0xFF) + (uint8_t)(rxPkt->shin_r  >> 8);
                chk += (uint8_t)(rxPkt->knee_r  & 0xFF) + (uint8_t)(rxPkt->knee_r  >> 8);
                chk += rxPkt->status;
                
                if (chk == rxPkt->checksum) {
                    // Valid packet! Send it over BLE
                    if (deviceConnected) {
                        pCharacteristic->setValue((uint8_t*)rxPkt, sizeof(EspTelemetry_t));
                        pCharacteristic->notify();
                    }
                } else {
                    Serial.println("UART Checksum failed!");
                }
                
                pktIndex = 0; // Reset for next packet
            }
        }
    }

    // Handle connection state changes
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}
