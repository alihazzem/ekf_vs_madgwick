#include <BLEDevice.h>
#include <BLEUtils.h>
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
#define STM32_TX_PIN   17
#define STM32_RX_PIN   16
#define STM32_BAUD     921600

#define LED_R          25
#define LED_G          26
#define LED_B          27

#define CALIB_BTN      32   // active LOW, internal pullup

// ─── EMG PROCESSING CONFIG ────────────────────────────────────────────────────
#define N_CH          8
#define RMS_WINDOW    15    // samples (~75ms at 200Hz)
#define UPDATE_EVERY  5     // send to STM32 every N samples (~25ms)

// ─── TEST MODE ────────────────────────────────────────────────────────────────
// Set to 1 to run without Myo armband (sends synthetic speed ramp via UART).
// Set to 0 for normal BLE operation with Myo armband.
#define TEST_MODE 0

// ─── CALIBRATION CONFIG ───────────────────────────────────────────────────────
#define CALIB_PREPARE_MS  2000
#define CALIB_DURATION_MS 4000

// ─── BLE UUIDs ────────────────────────────────────────────────────────────────
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

HardwareSerial STM32Serial(2);

// ─── DISPLAY VARIABLES ────────────────────────────────────────────────────────
// FIX 1: Deferred OLED update flags — BLE callbacks (Core 0) only write these;
//         actual I2C Wire transactions happen exclusively in loop() on Core 1.
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
// MUST only be called from loop() — never from a BLE callback.
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

// FIX 1: Safe wrapper for use inside BLE callbacks — no I2C, just flag + copy.
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
// FIX 1: All updateOLED() replaced with requestOledUpdate() — no I2C from Core 0.
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

// resetCalibration() is only ever called from loop() context so direct
// updateOLED() is safe here.
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

// ─── TEST MODE STATE ─────────────────────────────────────────────────────────
#if TEST_MODE
const uint8_t  testSpeeds[]    = {0, 100, 0, 100};
const uint8_t  testNumSteps    = sizeof(testSpeeds);
uint8_t        testStep        = 0;
unsigned long  testLastStepMs  = 0;
unsigned long  testLastPktMs   = 0;
const float    testFakeRMS[]   = {0.0f, 50.0f, 0.0f, 50.0f};
#endif

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
        STM32Serial.write(pkt, 3);

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
      // FIX 2: Free stale myoDevice before overwriting — prevents heap exhaustion
      //         across multiple reconnect cycles.
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
  scan->start(0, scanCompleteCB, false); // async, indefinite
}

// ─── CONNECT ─────────────────────────────────────────────────────────────────
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

  // ── FIX: Loop through and subscribe to ALL 4 EMG characteristics ───────────
  // This restores the full 200 Hz sampling rate by catching every data packet.
  bool atLeastOneEmgConnected = false;
  for (int i = 0; i < 4; i++) {
    BLERemoteCharacteristic* emgChar = emgService->getCharacteristic(emgCharUUIDs[i]);
    if (emgChar && emgChar->canNotify()) {
      emgChar->registerForNotify(emgCallback);
      atLeastOneEmgConnected = true;
    }
  }

  // If we couldn't connect to ANY of the EMG characteristics, abort.
  if (!atLeastOneEmgConnected) return false;

  lastDataTime = millis();
  resetCalibration(); // safe — called from loop() context
  return true;
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    updateOLED("Booting up...");
  }

  pinMode(LED_R,     OUTPUT);
  pinMode(LED_G,     OUTPUT);
  pinMode(LED_B,     OUTPUT);
  pinMode(CALIB_BTN, INPUT_PULLUP);

  setLED(false, false, false);

  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

#if TEST_MODE
  currentLED = LED_ACTIVE;
  setLED(false, true, false);  // green — test mode active
  updateOLED("TEST MODE");
  testLastStepMs = millis();
  testLastPktMs  = millis();
  debugSpeed = 0;
  debugRMS   = 0;
#else
  BLEDevice::init("ESP32_Myo");
  startScan();
#endif
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
#if TEST_MODE
  // ── TEST MODE: synthetic speed ramp ────────────────────────────────────────
  unsigned long now = millis();

  if (now - testLastStepMs >= 1000) {
    testStep = (testStep + 1) % testNumSteps;
    if (testStep == 0) testLastStepMs = now;              // avoid catch-up burst
    else               testLastStepMs += 1000;
  }

  if (now - testLastPktMs >= 25) {
    uint8_t speed = testSpeeds[testStep];
    uint8_t pkt[3] = { 0xAA, speed, (uint8_t)(0xAA ^ speed) };
    STM32Serial.write(pkt, 3);
    testLastPktMs = now;

    debugRMS   = testFakeRMS[testStep];
    debugSpeed = speed;
    newDebug   = true;
  }

  updateLED();

  if (needOledUpdate) {
    needOledUpdate = false;
    updateOLED(pendingOledMsg);
  }

  if (newDebug) {
    newDebug = false;
    static unsigned long lastOledUpdate = 0;
    if (now - lastOledUpdate > 500) {
      char buf[32];
      snprintf(buf, sizeof(buf), "TEST [%u/%u]", testStep + 1, testNumSteps);
      updateOLED(buf);
      lastOledUpdate = now;
    }
  }
  return;
#endif

  updateLED();

  // ── FIX 1: Flush deferred OLED updates from BLE callbacks ─────────────────
  if (needOledUpdate) {
    needOledUpdate = false;
    updateOLED(pendingOledMsg);
  }

  // ── Connect after successful scan ─────────────────────────────────────────
  if (myoFound && !scanning) {
    if (!connectToMyo()) {
      delay(2000);
      startScan();
    }
    myoFound = false;
  }

  // ── Reconnect on dropped connection ───────────────────────────────────────
  if (client != nullptr && !client->isConnected()) {
    uint8_t stopPkt[3] = { 0xAA, 0, (uint8_t)(0xAA ^ 0) };
    STM32Serial.write(stopPkt, 3);
    debugSpeed = 0;

    currentLED = LED_DISCONNECTED;
    updateOLED("Connection Lost!");

    delete client;
    client   = nullptr;
    myoFound = false;

    delay(1000);
    startScan();
  }

  // ── FIX 3: No-data watchdog — clear lastDataTime before disconnect ─────────
  // Prevents the stale timestamp from killing the fresh connection immediately
  // after a successful reconnect.
  if (client != nullptr && client->isConnected() &&
      lastDataTime != 0 && millis() - lastDataTime > 5000) {
    Serial.println("No data timeout!");
    lastDataTime = 0;     // ← FIX 3
    client->disconnect();
  }

  // ── Calibration button ────────────────────────────────────────────────────
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

  // ── Periodic OLED refresh while active ───────────────────────────────────
  if (newDebug) {
    newDebug = false;
    static unsigned long lastOledUpdate = 0;
    if (millis() - lastOledUpdate > 500) {
      updateOLED("Myo Active");
      lastOledUpdate = millis();
    }
  }
}