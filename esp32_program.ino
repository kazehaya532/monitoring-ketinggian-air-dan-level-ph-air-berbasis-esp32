#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include "HX711.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_INA219.h>  // Library untuk sensor INA219

#define LED_BUILTIN 2        // ESP32 built-in LED pin
#define RTC_SDA 21           // SDA pin for RTC
#define RTC_SCL 22           // SCL pin for RTC

// Pin HX711 definitions  
#define DOUT 4
#define SCK  18

// pH Sensor configuration
#define PH_PIN 35            // Pin analog dari Po modul PH4502C
#define ADC_RESOLUTION 4095.0 // 12-bit ADC pada ESP32
#define VREF 3.3             // Tegangan referensi ADC ESP32

// Kalibrasi: Ubah nilai ini berdasarkan hasil pengukuran larutan pH standar
#define PH4_VOLTAGE 2.55     // Tegangan saat probe dalam larutan pH 4.0 (dalam volt)
#define PH7_VOLTAGE 2.41     // Tegangan saat probe dalam larutan pH 7.0 (dalam volt)

// Wi-Fi and MQTT credentials
const char* ssid = "SISELJUT";
const char* password = "setubabakan";

// Emqx
const char* mqtt_server = "db90cf2e.ala.asia-southeast1.emqxsl.com";
const char* mqtt_user = "AMAR";
const char* mqtt_pass = "amar113118";

// NTP Server settings
const char *ntp_server = "pool.ntp.org";     // Default NTP server
const long gmt_offset_sec = 0 * 3600;            // GMT offset in seconds (GMT+7 for Jakarta)
const int daylight_offset_sec = 0;        // Daylight saving time offset in seconds

// WiFi and MQTT client initialization
WiFiClientSecure espClient;
PubSubClient client(espClient);

// SSL certificate for MQTT broker
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

RTC_DS3231 rtc;
Adafruit_INA219 ina219;  // Inisialisasi sensor INA219

String serial_number = "1101";
bool offlineDataSent = false;
unsigned long lastReconnectAttempt = 0;
bool otaInitialized = false;

HX711 scale;

// known calibration points for water level:
const long RAW_PTS[] = { -368848, -22958, 269206, 585644 };
const float CM_PTS[]  = {      0.0,      5.0,     10.0,     15.0 };
const int  N_PTS     = 4;

// After 15 cm (RAW ≥ 1970000), every +270000 raw → +5 cm
const long  RAW_STEP = 270000;
const float CM_STEP  = 5.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize LED pins (ESP32 built-in LED is on GPIO2)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set ADC resolution for pH sensor
  analogReadResolution(12);  // 12-bit ADC resolution for ESP32
  Serial.println("PH4502C Sensor Started");

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Initialize I2C on custom pins
  Wire.begin(RTC_SDA, RTC_SCL);

  // Initialize RTC DS3231
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  Serial.println("RTC is online!");

  // Initialize INA219 sensor
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("INA219 sensor initialized successfully!");

  // Initialize HX711 scale
  scale.begin(DOUT, SCK);
  scale.set_scale();  // you can still apply a scale factor here if needed
  scale.tare();
  Serial.println("HX711 → cm converter ready");

  // Wi-Fi setup - moved after hardware initialization
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // EMQX
  client.setServer(mqtt_server, 8883);

  // Initialize OTA
  setupOTA();
}

void setupOTA() {
  ArduinoOTA.setPassword("kelompokA1");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
}

void loop() {
  // Handle OTA if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaInitialized) {
      ArduinoOTA.begin();
      Serial.println("OTA Ready");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      otaInitialized = true;
    }
    ArduinoOTA.handle();
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("WiFi disconnected, attempting reconnection...");
    if (!reconnectWiFi()) {
      offlineMode();
      return; // Exit loop to restart connection process
    }
  }

  Serial.println("WiFi connected");
  
  // Sync time if needed
  if (!syncTime()) {
    Serial.println("Time sync failed, continuing anyway...");
  }
  
  // Handle MQTT connection
  if (!client.connected()) {
    Serial.println("MQTT disconnected, attempting reconnection...");
    if (!reconnectMQTT()) {
      Serial.println("MQTT reconnection failed, will retry next loop");
      delay(5000);
      return;
    }
  }
  
  // Main operation loop
  if (client.connected()) {
    client.loop();
    publishOnline();
    delay(3000);
  }
}

bool syncTime() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot sync time: WiFi not connected");
    return false;
  }

  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
  Serial.println("Waiting for NTP time sync: ");

  const int maxRetries = 10;    // Increased retry count
  int retryCount = 0;
  struct tm timeinfo;
  
  while (!getLocalTime(&timeinfo) && retryCount < maxRetries) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (retryCount >= maxRetries) {
    Serial.println("\n[!] NTP sync failed!");
    return false;
  }

  Serial.println("\nTime synchronized: ");
  Serial.print(asctime(&timeinfo));

  DateTime currentTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  rtc.adjust(currentTime);
  Serial.print("RTC time: ");
  Serial.println(currentTime.timestamp());
  return true;
}

bool reconnectWiFi() {
  // Avoid too frequent reconnection attempts
  if (millis() - lastReconnectAttempt < 5000) {
    return false;
  }
  lastReconnectAttempt = millis();
  
  Serial.println("Attempting WiFi reconnection...");
  WiFi.disconnect();
  delay(1000);
  WiFi.begin(ssid, password);
  
  unsigned long startTime = millis();
  const unsigned long timeout = 15000; // 15 second timeout
  
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi reconnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    otaInitialized = false; // Reset OTA flag to reinitialize
    return true;
  }

  Serial.println("\nWiFi reconnection failed.");
  return false;
}

void sendOfflineData() {
  // Check if the file exists and if offline data has not already been sent
  if (!SPIFFS.exists("/data.txt") || offlineDataSent) return;

  File file = SPIFFS.open("/data.txt", "r");
  if (!file) {
    Serial.println("Failed to open file SPIFFS for reading.");
    return;
  }

  Serial.println("Sending saved offline data to MQTT");
  int lineCount = 0;
  
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (client.publish("kelA1/offline", line.c_str())) {
        lineCount++;
        Serial.print("Sent line ");
        Serial.print(lineCount);
        Serial.print(": ");
        Serial.println(line);
      } else {
        Serial.println("Failed to publish offline data");
        file.close();
        return;
      }
      delay(100); // Small delay between publishes
    }
  }
  
  file.close();
  
  if (lineCount > 0) {
    SPIFFS.remove("/data.txt");
    offlineDataSent = true;
    Serial.print("Successfully sent ");
    Serial.print(lineCount);
    Serial.println(" offline records. File removed.");
  } else {
    Serial.println("No valid data found in offline file.");
  }
}

bool reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot connect to MQTT: WiFi not connected");
    return false;
  }

  Serial.println("Connecting to MQTT...");
  espClient.setCACert(ca_cert);
  
  if (client.connect("espA1", mqtt_user, mqtt_pass)) {
    Serial.println("MQTT connected successfully!");
    digitalWrite(LED_BUILTIN, LOW);
    
    // Publish online status
    if (client.publish("kelA1/status/1101", "online")) {
      Serial.println("Online status published");
    }
    
    // Send any offline data
    sendOfflineData();
    return true;
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(client.state());
    return false;
  }
}

float readPH() {
  int analogValue = analogRead(PH_PIN);
  float voltage = analogValue * VREF / ADC_RESOLUTION;

  // Linear interpolation between calibration points
  float slope = (7.0 - 4.0) / (PH7_VOLTAGE - PH4_VOLTAGE);  // ∆pH/∆V
  float intercept = 7.0 - slope * PH7_VOLTAGE;
  float phValue = slope * voltage + intercept;

  Serial.print("pH Analog: ");
  Serial.print(analogValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | pH: ");
  Serial.println(phValue, 2);

  return phValue;
}

void publishOnline() {
  if (!client.connected()) {
    Serial.println("Cannot publish: MQTT not connected");
    return;
  }

  // Read pH sensor value
  float phValue = readPH();

  // Read water level from HX711
  long raw = scale.read();
  float dist = rawToCm(raw);

  // Read voltage and current from INA219, calculate power
  float busVoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float powerValue = busVoltage * (current / 1000.0); // Convert mA to A, then calculate power in Watts

  // Create JSON object
  StaticJsonDocument<250> jsonDoc;
  jsonDoc["pressure"] = raw;
  jsonDoc["distance"] = dist;
  jsonDoc["ph"] = phValue;
  jsonDoc["Arus"] = current;

  char buffer[200];
  serializeJson(jsonDoc, buffer);

  if (client.publish("kelA1/online", buffer)) {
    Serial.print("Published: ");
    Serial.println(buffer);
  } else {
    Serial.println("Failed to publish data");
  }
}

void offlineMode() {
  Serial.println("Entering offline mode - WiFi unavailable");
  offlineDataSent = false; // Reset flag for next online session

  unsigned long offlineStart = millis();
  const unsigned long offlineTimeout = 60000; // Try to reconnect every minute
  
  while (WiFi.status() != WL_CONNECTED) {
    DateTime now = rtc.now();
    unsigned long timestamp = now.unixtime();

    // Read sensor values
    float phValue = readPH();
    long raw = scale.read();
    float dist = rawToCm(raw);
    
    // Calculate power for offline mode
    float busVoltage = ina219.getBusVoltage_V();
    float current = ina219.getCurrent_mA();
    float powerValue = busVoltage * (current / 1000.0);

    // Create JSON data
    String jsonData = "{\"timestamp\":" + String(timestamp) +
                     ",\"pressure\":" + String(raw) +
                     ",\"distance\":" + String(dist, 2) +
                     ",\"ph\":" + String(phValue, 2) +
                     ",\"Arus\":" + String(current, 2) + "}";

    saveDataToSPIFFS(jsonData);
    Serial.println("Saved offline: " + jsonData);
    
    // Try to reconnect periodically
    if (millis() - offlineStart > offlineTimeout) {
      Serial.println("Attempting to exit offline mode...");
      if (reconnectWiFi()) {
        Serial.println("Reconnected! Exiting offline mode.");
        return;
      }
      offlineStart = millis(); // Reset timeout
    }
    
    delay(5000); // Read sensors every 5 seconds in offline mode
  }
}

void saveDataToSPIFFS(String data) {
  if (data.length() == 0) {
    Serial.println("[ERROR] No data to save.");
    return;
  }

  File file = SPIFFS.open("/data.txt", "a");
  if (!file) {
    Serial.println("Failed to open SPIFFS for writing.");
    return;
  }

  file.println(data);
  file.close();
}

float interp(long x0, float y0, long x1, float y1, long x) {
  return y0 + (float)(y1 - y0) * (x - x0) / (float)(x1 - x0);
}

float rawToCm(long raw) {
  // 1) If below first point, clamp to 0cm
  if (raw <= RAW_PTS[0]) return CM_PTS[0];

  // 2) Between any two known points → linear interpolate
  for (int i = 0; i < N_PTS - 1; i++) {
    if (raw <= RAW_PTS[i+1]) {
      return interp(RAW_PTS[i], CM_PTS[i],
                    RAW_PTS[i+1], CM_PTS[i+1],
                    raw);
    }
  }

  // 3) Beyond last calibration point → extrapolate in steps
  long extra = raw - RAW_PTS[N_PTS-1];
  float cmExtra = ((float)extra / RAW_STEP) * CM_STEP;
  return CM_PTS[N_PTS-1] + cmExtra;
}
