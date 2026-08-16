// --- ESP32 FULL MODE: BLE SERVER (LEG TELEMETRY) + BLE CLIENT (MYO ARM) ---

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ─── OLED DISPLAY INCLUDES ────────────────────────────────────────────────────
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─── PIN CONFIG ───────────────────────────────────────────────────────────────
// Telemetry from STM32 USART6 (TX) to ESP32 Serial1 (RX)
#define TELEMETRY_RX_PIN 4
#define TELEMETRY_TX_PIN -1 // Unused
#define TELEMETRY_BAUD   115200

// EMG Commands from ESP32 Serial2 (TX) to STM32 USART1 (RX)
#define MYO_RX_PIN       -1 // Unused
#define MYO_TX_PIN       17
#define MYO_BAUD         921600

#define LED_R          25
#define LED_G          26
#define LED_B          27

#define CALIB_BTN      32   // active LOW, internal pullup

// ─── EMG PROCESSING CONFIG ────────────────────────────────────────────────────
#define N_CH          8
#define RMS_WINDOW    15    // samples (~75ms at 200Hz)
#define UPDATE_EVERY  5     // send to STM32 every N samples (~25ms)

// ─── CALIBRATION CONFIG ───────────────────────────────────────────────────────
#define CALIB_PREPARE_MS  2000
#define CALIB_DURATION_MS 4000

// ─── BLE SERVER UUIDS (Telemetry) ───────────────────────────────────────────────
#define TELEMETRY_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TELEMETRY_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ─── BLE CLIENT UUIDS (Myo) ────────────────────────────────────────────────────
#define CONTROL_SERVICE_UUID  "d5060001-a904-deb9-4748-2c7f4a124842"
#define EMG_SERVICE_UUID      "d5060005-a904-deb9-4748-2c7f4a124842"
#define IMU_SERVICE_UUID      "d5060002-a904-deb9-4748-2c7f4a124842"
#define IMU_DATA_UUID         "d5060402-a904-deb9-4748-2c7f4a124842"
#define COMMAND_UUID          "d5060401-a904-deb9-4748-2c7f4a124842"

BLEUUID emgCharUUIDs[4] = {
  BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060205-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060305-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060405-a904-deb9-4748-2c7f4a124842")
};

// ─── TELEMETRY STRUCT ─────────────────────────────────────────────────────────
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
    uint8_t status;            // 0=OK, 1=Lost Connection
    uint8_t checksum;
} EspTelemetry_t;
#pragma pack(pop)

uint8_t pktBuffer[sizeof(EspTelemetry_t)];
int pktIndex = 0;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ─── IIR FILTER COEFFICIENTS ─────────────────────────────────────────────────
const float SOS[2][6] = {
  { 0.62480f, -1.24960f, 0.62480f, 1.0f, -1.14948f,  0.29609f },
  { 1.00000f, -2.00000f, 1.00000f, 1.0f, -1.22534f,  0.64117f }
};
float sosState[2][N_CH][2] = {{{0}}};

// ─── CALIBRATION STATE ────────────────────────────────────────────────────────
enum CalibState { CALIB_PREPARE, CALIB_REST, CALIB_MAX, CALIB_DONE };
CalibState    calibState      = CALIB_PREPARE;
bool          calibStarted    = false;
float         calibAccum      = 0;
int           calibSamples    = 0;
unsigned long calibPhaseStart = 0;

float THR_ACTIVATE   = 0;
float THR_DEACTIVATE = 0;
float restRMS        = 0;

// ─── BLE STATE ────────────────────────────────────────────────────────────────
BLEClient* client       = nullptr;
BLEAdvertisedDevice* myoDevice    = nullptr;
bool                 myoFound     = false;
bool                 scanning     = false;
unsigned long        lastDataTime = 0;

HardwareSerial TelemetrySerial(1);
HardwareSerial MyoSerial(2);

// ─── DISPLAY VARIABLES ────────────────────────────────────────────────────────
volatile bool    newDebug       = false;
volatile float   debugRMS       = 0;
volatile uint8_t debugSpeed     = 0;

volatile bool    needOledUpdate = false;
char             pendingOledMsg[32] = "Booting up...";

// ─── LED STATE ────────────────────────────────────────────────────────────────
enum LEDState { LED_SCANNING, LED_PREPARE, LED_REST_PHASE, LED_FLEX_PHASE, LED_ACTIVE, LED_DISCONNECTED };
LEDState      currentLED  = LED_SCANNING;
unsigned long lastBlinkMs = 0;
bool          blinkToggle = false;

// ─── LED HELPERS (Common Anode) ───────────────────────────────────────────────
void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

void updateLED() {
  unsigned long now = millis();
  switch (currentLED) {
    case LED_SCANNING:
      if (now - lastBlinkMs > 600) {
        blinkToggle = !blinkToggle;
        lastBlinkMs = now;
        setLED(false, false, blinkToggle);
      }
      break;
    case LED_PREPARE:
      if (now - lastBlinkMs > 250) {
        blinkToggle = !blinkToggle;
        lastBlinkMs = now;
        setLED(blinkToggle, blinkToggle, false);
      }
      break;
    case LED_REST_PHASE: setLED(true, true, false);  break;
    case LED_FLEX_PHASE: setLED(true, false, false); break;
    case LED_ACTIVE:     setLED(false, true, false); break;
    case LED_DISCONNECTED:
      if (now - lastBlinkMs > 200) {
        blinkToggle = !blinkToggle;
        lastBlinkMs = now;
        setLED(blinkToggle, false, false);
      }
      break;
  }
}

// ─── OLED HELPER ─────────────────────────────────────────────────────────────
void updateOLED(const char* statusMsg) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(statusMsg);

  display.setCursor(0, 20);
  display.printf("A:%.1f D:%.1f", THR_ACTIVATE, THR_DEACTIVATE);

  display.setCursor(0, 35);
  display.print("RMS: ");
  display.print(debugRMS, 1);

  display.setCursor(0, 50);
  if (debugSpeed == 100) {
    display.print("GRIP: ACTIVE");
  } else {
    display.print("GRIP: RELAXED");
  }

  display.display();
}

void requestOledUpdate(const char* msg) {
  strncpy(pendingOledMsg, msg, sizeof(pendingOledMsg) - 1);
  pendingOledMsg[sizeof(pendingOledMsg) - 1] = '\0';
  needOledUpdate = true;
}

// ─── BIQUAD FILTER ────────────────────────────────────────────────────────────
float applyHighpass(float x, int ch) {
  for (int s = 0; s < 2; s++) {
    float b0 = SOS[s][0], b1 = SOS[s][1], b2 = SOS[s][2];
    float a1 = SOS[s][4], a2 = SOS[s][5];
    float w  = x - a1 * sosState[s][ch][0] - a2 * sosState[s][ch][1];
    float y  = b0 * w + b1 * sosState[s][ch][0] + b2 * sosState[s][ch][1];
    sosState[s][ch][1] = sosState[s][ch][0];
    sosState[s][ch][0] = w;
    x = y;
  }
  return x;
}

// ─── SPEED QUANTIZATION ──────────────────────────────────────────────────────
uint8_t rmsToSpeed(float rms) {
  static uint8_t lastSpeed = 0;

  if (lastSpeed == 0) {
    if (rms >= THR_ACTIVATE) {
      lastSpeed = 100;
    }
  } else {
    if (rms <= THR_DEACTIVATE) {
      lastSpeed = 0;
    }
  }
  return lastSpeed;
}

// ─── CALIBRATION HANDLER ─────────────────────────────────────────────────────
void handleCalibration(float rms) {

  if (calibState == CALIB_PREPARE) {
    if (!calibStarted) {
      currentLED      = LED_PREPARE;
      calibPhaseStart = millis();
      calibStarted    = true;
      requestOledUpdate("Get Ready... (2s)");
    }
    if (millis() - calibPhaseStart >= CALIB_PREPARE_MS) {
      calibState   = CALIB_REST;
      calibStarted = false;
    }
  }

  else if (calibState == CALIB_REST) {
    if (!calibStarted) {
      currentLED      = LED_REST_PHASE;
      calibPhaseStart = millis();
      calibStarted    = true;
      calibAccum      = 0;
      calibSamples    = 0;
      requestOledUpdate("Calibrate: RELAX");
    }
    calibAccum += rms;
    calibSamples++;

    if (millis() - calibPhaseStart >= CALIB_DURATION_MS) {
      restRMS      = calibAccum / calibSamples;
      calibState   = CALIB_MAX;
      calibStarted = false;
    }
  }

  else if (calibState == CALIB_MAX) {
    if (!calibStarted) {
      currentLED      = LED_FLEX_PHASE;
      calibPhaseStart = millis();
      calibStarted    = true;
      calibAccum      = 0;
      calibSamples    = 0;
      requestOledUpdate("Calibrate: FLEX!");
    }
    calibAccum += rms;
    calibSamples++;

    if (millis() - calibPhaseStart >= CALIB_DURATION_MS) {
      float flexRMS = calibAccum / calibSamples;
      float range   = flexRMS - restRMS;

      THR_DEACTIVATE = restRMS + range * 0.30f;
      THR_ACTIVATE   = restRMS + range * 0.65f;

      calibState = CALIB_DONE;
      currentLED = LED_ACTIVE;
      requestOledUpdate("Myo Active");
    }
  }
}

void resetCalibration() {
  calibState   = CALIB_PREPARE;
  calibStarted = false;
  calibAccum   = 0;
  calibSamples = 0;
  currentLED   = LED_PREPARE;
  updateOLED("Get Ready... (2s)");
}

// ─── RMS ACCUMULATORS ────────────────────────────────────────────────────────
float    rmsAccum[N_CH] = {0};
uint16_t sampleCount    = 0;

// ─── EMG CALLBACK ────────────────────────────────────────────────────────────
void emgCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  if (length < 16) return;
  lastDataTime = millis();

  for (int s = 0; s < 2; s++) {
    for (int ch = 0; ch < N_CH; ch++) {
      float raw      = (float)((int8_t)data[s * 8 + ch]);
      float filtered = applyHighpass(raw, ch);
      float rect     = fabsf(filtered);
      rmsAccum[ch]  += rect * rect;
    }
    sampleCount++;

    if (sampleCount >= RMS_WINDOW && sampleCount % UPDATE_EVERY == 0) {
      float avgRMS = 0;
      for (int ch = 0; ch < N_CH; ch++) {
        avgRMS       += sqrtf(rmsAccum[ch] / RMS_WINDOW);
        rmsAccum[ch] *= (float)(RMS_WINDOW - UPDATE_EVERY) / RMS_WINDOW;
      }
      avgRMS /= N_CH;

      if (calibState != CALIB_DONE) {
        handleCalibration(avgRMS);
      } else {
        uint8_t speed = rmsToSpeed(avgRMS);
        uint8_t pkt[3] = { 0xAA, speed, (uint8_t)(0xAA ^ speed) };
        MyoSerial.write(pkt, 3);

        debugRMS   = avgRMS;
        debugSpeed = speed;
        newDebug   = true;
      }

      sampleCount = 0;
      for (int ch = 0; ch < N_CH; ch++) rmsAccum[ch] = 0;
    }
  }
}

void imuCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  lastDataTime = millis();
}

// ─── BLE SCAN CALLBACKS ──────────────────────────────────────────────────────
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.isAdvertisingService(BLEUUID(CONTROL_SERVICE_UUID))) {
      if (myoDevice != nullptr) {
        delete myoDevice;
        myoDevice = nullptr;
      }
      myoDevice = new BLEAdvertisedDevice(advertisedDevice);
      myoFound  = true;
      scanning  = false;
      BLEDevice::getScan()->stop();
    }
  }
};

void scanCompleteCB(BLEScanResults scanResults) {}

void startScan() {
  if (scanning) return;
  currentLED = LED_SCANNING;
  scanning   = true;
  myoFound   = false;
  updateOLED("Scanning for Myo...");
  BLEScan* scan = BLEDevice::getScan();
  scan->clearResults();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  scan->setActiveScan(true);
  scan->start(0, scanCompleteCB, false); 
}

// ─── CONNECT TO MYO ──────────────────────────────────────────────────────────
bool connectToMyo() {
  if (client != nullptr) {
    client->disconnect();
    delete client;
    client = nullptr;
  }

  updateOLED("Connecting...");
  client = BLEDevice::createClient();
  if (!client->connect(myoDevice)) return false;

  BLERemoteService* controlService = client->getService(CONTROL_SERVICE_UUID);
  if (!controlService) return false;

  BLERemoteCharacteristic* commandChar = controlService->getCharacteristic(COMMAND_UUID);
  if (!commandChar) return false;

  // Send the command to enable EMG (and IMU if configured)
  uint8_t cmdEMG[] = {0x01, 0x03, 0x03, 0x01, 0x01};
  commandChar->writeValue(cmdEMG, sizeof(cmdEMG));

  // Connect IMU if available
  BLERemoteService* imuService = client->getService(IMU_SERVICE_UUID);
  if (imuService) {
    BLERemoteCharacteristic* imuChar = imuService->getCharacteristic(IMU_DATA_UUID);
    if (imuChar && imuChar->canNotify()) {
        imuChar->registerForNotify(imuCallback);
    }
  }

  // Connect EMG Service
  BLERemoteService* emgService = client->getService(EMG_SERVICE_UUID);
  if (!emgService) return false;

  bool atLeastOneEmgConnected = false;
  for (int i = 0; i < 4; i++) {
    BLERemoteCharacteristic* emgChar = emgService->getCharacteristic(emgCharUUIDs[i]);
    if (emgChar && emgChar->canNotify()) {
      emgChar->registerForNotify(emgCallback);
      atLeastOneEmgConnected = true;
    }
  }

  if (!atLeastOneEmgConnected) return false;

  lastDataTime = millis();
  resetCalibration(); 
  return true;
}

// ─── BLE SERVER CALLBACKS ────────────────────────────────────────────────────
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      deviceConnected = true;
      Serial.println("Telemetry Client connected");
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
      Serial.println("Telemetry Client disconnected");
    }
};

// ─── PROCESS TELEMETRY TASK ──────────────────────────────────────────────────
// Running this in its own task on Core 0 ensures that blocking delays in loop()
// (like Myo connection retries or OLED updates) NEVER delay telemetry parsing.
void telemetryTask(void *pvParameters) {
  unsigned long lastUartByteTime = millis();
  
  for (;;) {
    bool bytesReceived = false;
    while (TelemetrySerial.available() > 0) {
      uint8_t b = TelemetrySerial.read();
      lastUartByteTime = millis();
      bytesReceived = true;
      
      if (pktIndex == 0) {
          if (b == 0xAA) pktBuffer[pktIndex++] = b;
      } else if (pktIndex == 1) {
          if (b == 0x55) pktBuffer[pktIndex++] = b;
          else pktIndex = 0;
      } else if (pktIndex < sizeof(EspTelemetry_t)) {
          pktBuffer[pktIndex++] = b;
          
          if (pktIndex == sizeof(EspTelemetry_t)) {
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
                  if (deviceConnected) {
                      pCharacteristic->setValue((uint8_t*)rxPkt, sizeof(EspTelemetry_t));
                      pCharacteristic->notify();
                  }
              }
              pktIndex = 0;
          }
      }
    }
    
    // Failsafe: if we haven't received telemetry from STM32 for >100ms, send a fault packet
    if (deviceConnected && (millis() - lastUartByteTime > 100)) {
        static unsigned long lastFaultSend = 0;
        if (millis() - lastFaultSend > 50) { // Send fault packet at ~20Hz
            EspTelemetry_t faultPkt;
            memset(&faultPkt, 0, sizeof(faultPkt));
            faultPkt.header1 = 0xAA;
            faultPkt.header2 = 0x55;
            faultPkt.status = 1; // 1 = Lost Connection
            
            uint8_t chk = faultPkt.header1 + faultPkt.header2 + faultPkt.status;
            faultPkt.checksum = chk;
            
            pCharacteristic->setValue((uint8_t*)&faultPkt, sizeof(EspTelemetry_t));
            pCharacteristic->notify();
            lastFaultSend = millis();
        }
    }

    // Yield for 2ms to ensure we process UART fast enough for a 200Hz (5ms) stream
    vTaskDelay(2 / portTICK_PERIOD_MS); 
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    updateOLED("Booting FULL MODE...");
  }

  pinMode(LED_R,     OUTPUT);
  pinMode(LED_G,     OUTPUT);
  pinMode(LED_B,     OUTPUT);
  pinMode(CALIB_BTN, INPUT_PULLUP);

  setLED(false, false, false);

  // Initialize UARTS
  TelemetrySerial.begin(TELEMETRY_BAUD, SERIAL_8N1, TELEMETRY_RX_PIN, TELEMETRY_TX_PIN);
  MyoSerial.begin(MYO_BAUD, SERIAL_8N1, MYO_RX_PIN, MYO_TX_PIN);

  // Initialize BLE Device
  BLEDevice::init("ESP32_FULL_MODE");

  // --- SETUP BLE SERVER (TELEMETRY) ---
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(TELEMETRY_SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      TELEMETRY_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(TELEMETRY_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
  BLEDevice::startAdvertising();

  // Create Telemetry Task on Core 0 to run independently from loop (Core 1)
  xTaskCreatePinnedToCore(
    telemetryTask,
    "TelemetryTask",
    4096, // Stack size
    NULL,
    1,    // Priority
    NULL,
    0     // Core 0
  );

  // --- SETUP BLE CLIENT (MYO) ---
  startScan();
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  updateLED();

  // Handle Server Re-advertising
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  // Handle OLED 
  if (needOledUpdate) {
    needOledUpdate = false;
    updateOLED(pendingOledMsg);
  }

  // Handle Myo Connection
  if (myoFound && !scanning) {
    if (!connectToMyo()) {
      delay(2000);
      startScan();
    }
    myoFound = false;
  }

  // Handle Myo Disconnect
  if (client != nullptr && !client->isConnected()) {
    uint8_t stopPkt[3] = { 0xAA, 0, (uint8_t)(0xAA ^ 0) };
    MyoSerial.write(stopPkt, 3);
    debugSpeed = 0;

    currentLED = LED_DISCONNECTED;
    updateOLED("Connection Lost!");

    delete client;
    client   = nullptr;
    myoFound = false;

    delay(1000);
    startScan();
  }

  // Myo Watchdog
  if (client != nullptr && client->isConnected() &&
      lastDataTime != 0 && millis() - lastDataTime > 5000) {
    Serial.println("No data timeout!");
    lastDataTime = 0;     
    client->disconnect();
  }

  // Calibration Button
  if (digitalRead(CALIB_BTN) == LOW) {
    delay(50);
    if (digitalRead(CALIB_BTN) == LOW) {
      if (client != nullptr && client->isConnected()) {
        resetCalibration();
      } else {
        updateOLED("Connect Myo First!");
        delay(1500);
        updateOLED("Scanning for Myo...");
      }
      while (digitalRead(CALIB_BTN) == LOW);
    }
  }

  // OLED Refresh
  if (newDebug) {
    newDebug = false;
    static unsigned long lastOledUpdate = 0;
    if (millis() - lastOledUpdate > 500) {
      updateOLED("Myo Active");
      lastOledUpdate = millis();
    }
  }
}
