/*
  esp32_multisensor_spo2_aws.ino
  Streams temperature (DS18B20), SpO2 + HR (MAX30105), ECG (AD8232)
  Publishes JSON to AWS IoT Core via MQTT.
*/

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include "secrets.h"  // contains Wi-Fi + AWS credentials & certs

// ------------------- Pins -------------------
#define ONE_WIRE_BUS 4
#define ECG_PIN 34

// ------------------- Constants -------------------
const uint16_t ECG_SAMPLE_RATE = 250; // Hz
const uint16_t ECG_BUFFER_LEN = 250;  // 1 second buffer
const uint32_t IR_THRESHOLD = 20000;  // Minimum IR value for valid finger detection

// ------------------- Globals -------------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MAX30105 particleSensor;

// ECG buffer
float ecgBuffer[ECG_BUFFER_LEN];
uint16_t ecgWriteIndex = 0;

// Heart rate and SpO2
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
uint32_t irBuffer[100];
uint32_t redBuffer[100];
const int bufferLength = 100;

// Timing
unsigned long ecgLastMicros = 0;
const unsigned long ecgIntervalUs = 1000000UL / ECG_SAMPLE_RATE;
unsigned long lastReportMillis = 0;
float lastTempC = NAN;

// --- WiFi + MQTT ---
WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
unsigned long lastMQTTReconnect = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;

// --------------------------------------------------
// MAX30105 Setup
// --------------------------------------------------
bool setupMAX30105() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring/power.");
    return false;
  }

  // Recommended settings for HR + SpO2
  byte ledBrightness = 60;  // 0 = off, 255 = max
  byte sampleAverage = 4;
  byte ledMode = 2;         // Red + IR
  byte sampleRate = 100;    // 100 samples/sec
  int pulseWidth = 411;     // 411µs
  int adcRange = 4096;      // 4096nA full scale

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.clearFIFO();
  Serial.println("MAX30105 initialized.");
  return true;
}

// --------------------------------------------------
// MQTT Reconnect
// --------------------------------------------------
void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT... ");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5s...");
      delay(5000);
    }
  }
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  setupMAX30105();

  for (uint16_t i = 0; i < ECG_BUFFER_LEN; ++i)
    ecgBuffer[i] = 0.0f;

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" ✅ Connected");

  // Time Sync
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" ✅ Synced");

  // TLS
  secureClient.setCACert(AWS_ROOT_CA);
  secureClient.setCertificate(AWS_DEVICE_CERT);
  secureClient.setPrivateKey(AWS_PRIVATE_KEY);

  // MQTT
  mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqttClient.setBufferSize(8192);
  mqttClient.setKeepAlive(60);

  connectToMQTT();

  ecgLastMicros = micros();
  lastReportMillis = millis();
}

// --------------------------------------------------
// Main Loop
// --------------------------------------------------
void loop() {
  if (!mqttClient.connected()) connectToMQTT();
  mqttClient.loop();

  unsigned long nowMicros = micros();
  unsigned long nowMs = millis();

  // ECG sampling
  if ((nowMicros - ecgLastMicros) >= ecgIntervalUs) {
    ecgLastMicros += ecgIntervalUs;
    int raw = analogRead(ECG_PIN);
    float voltage = (raw / 4095.0f) * 3.3f;
    ecgBuffer[ecgWriteIndex++] = voltage;
    if (ecgWriteIndex >= ECG_BUFFER_LEN) ecgWriteIndex = 0;
  }

  // HR + SpO2 sampling
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.check()) ;
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
  }

  // Compute HR & SpO2
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength,
    redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );

  // Report every 5 seconds
  if (nowMs - lastReportMillis >= 2000) {
    lastReportMillis = nowMs;

    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    
    if (tempC != DEVICE_DISCONNECTED_C && tempC > -55.0 && tempC < 125.0) {
      lastTempC = tempC;
    } else if (isnan(lastTempC) || lastTempC == -127.0) {
      lastTempC = 25.0;
    }

    // Create JSON payload
    String payload = "{";
    payload += "\"temperature_c\":" + String(lastTempC, 2) + ",";
    payload += "\"heart_rate\":" + String(validHeartRate ? heartRate : 0) + ",";
    payload += "\"spo2\":" + String(validSPO2 ? spo2 : 0) + ",";
    payload += "\"ecg\":[";
    for (uint16_t i = 0; i < 10; i++) {
      payload += String(ecgBuffer[i], 3);
      if (i < 9) payload += ",";
    }
    payload += "]}";

    Serial.println(payload);
    mqttClient.publish(AWS_IOT_TOPIC, payload.c_str());
  }
}
