#define BLYNK_TEMPLATE_ID "TMPL6BcIXK908"
#define BLYNK_TEMPLATE_NAME "IoT Project"
#define BLYNK_AUTH_TOKEN "3PLpTKpeBSAiO7_7SZXcuOiCGAlKKetZ"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Wi-Fi credentials
char ssid[] = "DARK PHOENIX";
char pass[] = "123asd07a";

// Firebase setup
#define API_KEY "AIzaSyASOLXA-khPFKOGeMqyfR_c8moY_PAhcnY"
#define DATABASE_URL "iot-project-1639f-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson dataJson;
bool signupOK = false;

// Virtual pins for Blynk
#define V1 1  // Heart Rate
#define V2 2  // SpO2
#define V3 3  // Systolic BP
#define V4 4  // Diastolic BP
#define V5 5  // Body Temp
#define V6 6  // Room Temp
#define V7 7  // Humidity
#define V8 8  // ECG

// SoftwareSerial for Mega communication
#define NODEMCU_TX D1
#define NODEMCU_RX D2
SoftwareSerial MegaSerial(NODEMCU_RX, NODEMCU_TX);

// NTP setup
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 3600000);

unsigned long lastReceivedTime = 0;
const unsigned long noDataInterval = 5000;

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);
  MegaSerial.begin(9600);

  // Blynk and Wi-Fi initialization
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Firebase initialization
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
    Serial.println("Firebase signup successful");
  } else {
    Serial.printf("Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // NTP initialization
  timeClient.begin();
  timeClient.setTimeOffset(19800);  // Adjust for UTC+5:30
}

void loop() {
  // Sync time
  timeClient.update();
  setTime(timeClient.getEpochTime());
  String currentTime = String(hour()) + ":" + String(minute()) + ":" + String(second());
  String currentDate = String(day()) + "/" + String(month()) + "/" + String(year());

  if (MegaSerial.available()) {
    String receivedData = MegaSerial.readStringUntil('\n');
    Serial.println("Received from Mega: " + receivedData);
    lastReceivedTime = millis();

    // Parse and handle data
    if (receivedData.indexOf("HeartRate:") != -1) {
      String heartRate = receivedData.substring(receivedData.indexOf("HeartRate:") + 10, receivedData.indexOf(",SpO2"));
      int heartRateValue = heartRate.toInt();
      Blynk.virtualWrite(V1, heartRateValue);
      dataJson.add("heartRate", heartRateValue);
    }

    if (receivedData.indexOf("SpO2:") != -1) {
      String spo2 = receivedData.substring(receivedData.indexOf("SpO2:") + 5, receivedData.indexOf(",SystolicBP"));
      double spo2Value = spo2.toFloat();
      Blynk.virtualWrite(V2, spo2Value);
      dataJson.add("spo2", spo2Value);
    }

    if (receivedData.indexOf("SystolicBP:") != -1) {
      String systolicBP = receivedData.substring(receivedData.indexOf("SystolicBP:") + 11, receivedData.indexOf(",DiastolicBP"));
      double systolicBPValue = systolicBP.toFloat();
      Blynk.virtualWrite(V3, systolicBPValue);
      dataJson.add("systolicBP", systolicBPValue);
    }

    if (receivedData.indexOf("DiastolicBP:") != -1) {
      String diastolicBP = receivedData.substring(receivedData.indexOf("DiastolicBP:") + 12);
      double diastolicBPValue = diastolicBP.toFloat();
      Blynk.virtualWrite(V4, diastolicBPValue);
      dataJson.add("diastolicBP", diastolicBPValue);
    }

    if (receivedData.indexOf("Body Temp:") != -1) {
      String bodyTemp = receivedData.substring(receivedData.indexOf("Body Temp:") + 10, receivedData.indexOf(","));
      double bodyTempValue = bodyTemp.toFloat();
      Blynk.virtualWrite(V5, bodyTempValue);
      dataJson.add("bodyTemp", bodyTempValue);
    }

    if (receivedData.indexOf("Room Temp:") != -1) {
      String roomTemp = receivedData.substring(receivedData.indexOf("Room Temp:") + 10, receivedData.indexOf(", Humidity"));
      double roomTempValue = roomTemp.toFloat();
      Blynk.virtualWrite(V6, roomTempValue);
      dataJson.add("roomTemp", roomTempValue);
    }

    if (receivedData.indexOf("Humidity:") != -1) {
      String humidity = receivedData.substring(receivedData.indexOf("Humidity:") + 9);
      double humidityValue = humidity.toFloat();
      Blynk.virtualWrite(V7, humidityValue);
      dataJson.add("humidity", humidityValue);
    }

    // Add time and date to Firebase JSON
    dataJson.add("time", currentTime);
    dataJson.add("date", currentDate);

    // Push data to Firebase
    if (Firebase.RTDB.pushJSON(&fbdo, "SensorData", &dataJson)) {
      Serial.println("Data sent to Firebase");
    } else {
      Serial.println("Failed to send data to Firebase: " + fbdo.errorReason());
    }

    dataJson.clear();  // Clear the JSON object for the next iteration
  }

  if (millis() - lastReceivedTime > noDataInterval) {
    Serial.println("No data received from Arduino Mega in the last 10 seconds");
    lastReceivedTime = millis();
  }

  Blynk.run();
}
