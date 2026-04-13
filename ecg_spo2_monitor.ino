#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"

// *** BLE LIBRARY ***
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =======================
// KONFIGURASI WIFI
// =======================
#define WIFI_SSID "Fayra"
#define WIFI_PASSWORD "01072022"

// =======================
// KONFIGURASI FIREBASE
// =======================
#define DATABASE_URL "https://ecg-iot-c536b-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define DATABASE_SECRET "lWGayIfmMH9wFAQF54iaIwhy6LCKacZCGtUc448X"

// =======================
// KONFIGURASI SENSOR
// =======================
#define ECG_PIN    34
#define LO_PLUS    32
#define LO_MINUS   33

MAX30105 particleSensor;
bool max30102_ready = false;

// =======================
// VARIABEL GLOBAL SENSOR
// =======================
float heart_rate = 0;
float hr_spo2 = 0;
float spo2 = 0;
unsigned long lastBeatOptical = 0;

unsigned long lastBeatTime = 0;
const int SAMPLE_RATE = 10;  // 100 Hz untuk ECG
unsigned long lastSampleTime = 0;

// SpO2 Variables
double avered = 0, aveir = 0;
double sumredrms = 0, sumirrms = 0;
int spo2_sample_count = 0;
double ESpO2 = 95.0;
double FSpO2 = 0.7;
double frate = 0.95;
#define TIMETOBOOT 3000
#define FINGER_ON 50000
#define MINIMUM_SPO2 0.0
#define NUM_SAMPLES 100

// =======================
// BLE CONFIG
// =======================
#define SERVICE_UUID            "12345678-1234-5678-9abc-123456789abc"
#define HR_CHARACTERISTIC_UUID  "12345678-1234-5678-9abc-123456789abd"
#define SPO2_CHARACTERISTIC_UUID "12345678-1234-5678-9abc-123456789abe"

BLECharacteristic *pHRCharacteristic;
BLECharacteristic *pSpO2Characteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected!");
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected. Restarting advertising...");
        BLEDevice::startAdvertising();
    }
};

void initBLE() {
    BLEDevice::init("ECG_SpO2_Monitor");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pHRCharacteristic = pService->createCharacteristic(
                        HR_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                    );
    pHRCharacteristic->addDescriptor(new BLE2902());

    pSpO2Characteristic = pService->createCharacteristic(
                        SPO2_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                    );
    pSpO2Characteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started.");
}

// =======================
// KIRIM DATA KE BLE
// =======================
void sendDataBLE(int ecgRaw) {
    if (deviceConnected) {
        String ecgStr = "E" + String(ecgRaw);
        pHRCharacteristic->setValue(ecgStr.c_str());
        pHRCharacteristic->notify();

        if (heart_rate > 0) {
            String hrStr = "HR:" + String(heart_rate, 1);
            pHRCharacteristic->setValue(hrStr.c_str());
            pHRCharacteristic->notify();
        }

        if (spo2 > 0) {
            String spo2Str = "SpO2:" + String(spo2, 1);
            pSpO2Characteristic->setValue(spo2Str.c_str());
            pSpO2Characteristic->notify();
        }
    }
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 IoT ECG + SpO₂ + Firebase ===");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan ke WiFi");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi tersambung!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi gagal. Lanjut tanpa Firebase.");
  }

  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  analogReadResolution(12);
  Serial.println("AD8232 siap.");

  Wire.begin(21, 22);
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    particleSensor.setup(255, 4, 2, 400, 411, 16384);
    particleSensor.setPulseAmplitudeRed(0x3F);
    particleSensor.setPulseAmplitudeIR(0x3F);
    max30102_ready = true;
    Serial.println("MAX30102 siap.");
  } else {
    Serial.println("MAX30102 tidak terdeteksi!");
  }

  initBLE();

  Serial.println("\nFormat Data (Serial):");
  Serial.println("E2048 (100x/detik) | Heart Rate: 78.5 | SpO2: 98.2");
  Serial.println("--------------------------------");
}

// =======================
// BACA HR DARI AD8232 + KIRIM ECG & HR
// =======================
float bacaHR_AD8232(int &ecgRawOut) {
  static float lastValue = 0;
  static bool peakDetected = false;
  static unsigned long lastPeakTime = 0;
  static float bpm = 0;
  float currentBpm = 0;

  ecgRawOut = 0;

  if (digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH) {
    Serial.println("Lead-off terdeteksi!");
    if (deviceConnected) {
      pHRCharacteristic->setValue("!");
      pHRCharacteristic->notify();
    }
    return 0;
  }

  if (millis() - lastSampleTime >= SAMPLE_RATE) {
    lastSampleTime = millis();
    
    int rawValue = analogRead(ECG_PIN);
    float currentValue = (float)rawValue;
    ecgRawOut = rawValue;

    // KIRIM ECG SETIAP SAMPEL (100 Hz)
    Serial.print("E");
    Serial.println(rawValue);

    // Deteksi peak
    if (currentValue > 2500 && currentValue > lastValue + 100) {
      if (!peakDetected) {
        unsigned long currentTime = millis();
        if (lastPeakTime > 0) {
          unsigned long interval = currentTime - lastPeakTime;
          if (interval > 300 && interval < 2000) {
            bpm = 60000.0 / interval;
            static float smoothedBpm = 0;
            if (smoothedBpm == 0) smoothedBpm = bpm;
            else smoothedBpm = 0.7 * smoothedBpm + 0.3 * bpm;
            currentBpm = smoothedBpm;
            if (currentBpm < 40 || currentBpm > 180) currentBpm = 0;
          }
        }
        lastPeakTime = currentTime;
        peakDetected = true;

        // KIRIM HR SAAT ADA UPDATE
        if (currentBpm > 0 && abs(heart_rate - currentBpm) > 0.1) {
          heart_rate = currentBpm;
          Serial.print("Heart Rate: ");
          Serial.println(heart_rate, 1);
        }
      }
    } else {
      peakDetected = false;
    }
    
    lastValue = currentValue;

    // Reset jika tidak ada detak dalam 3 detik
    if (millis() - lastPeakTime > 3000) {
      if (heart_rate > 0) {
        heart_rate = 0;
        Serial.println("Heart Rate: 0.0");
      }
      currentBpm = 0;
      bpm = 0;
    }
  }

  return heart_rate;
}

// =======================
// BACA SpO₂ + KIRIM SpO2 SETIAP UPDATE (100 sampel)
// =======================
void bacaSpO2_MAX30102(float &spo2_out, float &hr) {
  if (!max30102_ready) { spo2_out = 0; hr = 0; return; }

  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Jari tidak terdeteksi
  if (irValue < FINGER_ON || redValue < FINGER_ON) { 
    if (spo2 > 0) {
      spo2 = 0;
      Serial.println("SpO2: 0.0");
    }
    spo2_out = MINIMUM_SPO2; 
    hr = 0; 
    return; 
  }

  // Deteksi detak dari IR
  if (checkForBeat(irValue)) {
    unsigned long delta = millis() - lastBeatOptical;
    lastBeatOptical = millis();
    hr = 60.0 / (delta / 1000.0);
    if (hr < 40 || hr > 180) hr = 0;
  }

  double fred = (double)redValue;
  double fir = (double)irValue;
  
  avered = avered * frate + fred * (1.0 - frate);
  aveir = aveir * frate + fir * (1.0 - frate);
  
  sumredrms += (fred - avered) * (fred - avered);
  sumirrms += (fir - aveir) * (fir - aveir);
  spo2_sample_count++;

  if (spo2_sample_count % NUM_SAMPLES == 0 && millis() > TIMETOBOOT) {
    double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
    double SpO2 = -23.3 * (R - 0.4) + 100;
    ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
    spo2_out = ESpO2;
    if (spo2_out <= -1) spo2_out = 0;
    if (spo2_out > 100) spo2_out = 100;
    
    // KIRIM SETIAP UPDATE (setiap 100 sampel)
    spo2 = spo2_out;
    Serial.print("SpO2: ");
    Serial.println(spo2, 1);
    
    sumredrms = 0.0;
    sumirrms = 0.0;
    spo2_sample_count = 0;
  } else {
    spo2_out = ESpO2;
  }
}

// =======================
// LOOP UTAMA
// =======================
void loop() {
  int ecgRaw = 0;
  heart_rate = bacaHR_AD8232(ecgRaw);

  if (max30102_ready)
    bacaSpO2_MAX30102(spo2, hr_spo2);

  // 1. BLE: Kirim ECG + HR + SpO2 (20 Hz)
  static unsigned long lastBLESend = 0;
  if (millis() - lastBLESend > 50) {
    lastBLESend = millis();
    sendDataBLE(ecgRaw);
  }

  // 2. Serial: HR & SpO2 dikirim langsung di fungsi bacaHR & bacaSpO2
  //    → TIDAK DIPANGGIL DI SINI!

  // 3. Firebase: Kirim setiap 1 detik
  static unsigned long lastFirebase = 0;
  if (millis() - lastFirebase > 1000 && WiFi.status() == WL_CONNECTED) {
    lastFirebase = millis();
    
    HTTPClient http;
    String url = String(DATABASE_URL) + "/health_data.json?auth=" + DATABASE_SECRET;
    
    String payload = "{";
    payload += "\"heart_rate\":" + String(heart_rate, 1) + ",";
    payload += "\"spo2\":" + String(spo2, 1) + ",";
    payload += "\"ecg_raw\":" + String(ecgRaw) + ",";
    payload += "\"timestamp\":" + String(millis());
    payload += "}";

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(payload);
    if (code > 0) {
      Serial.printf("Firebase OK (%d)\n", code);
    } else {
      Serial.printf("Firebase error: %s\n", http.errorToString(code).c_str());
    }
    http.end();
  }

  delay(1);
}