#define BLYNK_TEMPLATE_ID "TMPL6BcIXK908"
#define BLYNK_TEMPLATE_NAME "IoT Project"
#define BLYNK_AUTH_TOKEN "3PLpTKpeBSAiO7_7SZXcuOiCGAlKKetZ"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>  // Include SoftwareSerial library

// WiFi credentials
char ssid[] = "DARK PHOENIX";
char pass[] = "1234567890.........";

// Virtual pins for Blynk
#define V1 1  // Heart Rate
#define V2 2  // SpO2
#define V3 3  // Systolic BP
#define V4 4  // Diastolic BP
#define V5 5  // Body Temp
#define V6 6  // Room Temp
#define V7 7  // Humidity
#define V8 8  // ECG

// Declare SoftwareSerial pins
#define NODEMCU_TX D1  // GPIO5
#define NODEMCU_RX D2  // GPIO4

// Create SoftwareSerial object
SoftwareSerial MegaSerial(NODEMCU_RX, NODEMCU_TX);  // RX, TX

unsigned long lastReceivedTime = 0;         // Time of last data received
const unsigned long noDataInterval = 5000;  // 5 seconds

void setup() {
  // Start Serial communication for debugging
  Serial.begin(9600);

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize SoftwareSerial for Mega communication
  MegaSerial.begin(9600);

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  Serial.println("NodeMCU ready!");
}

void loop() {
  // Check if data is available from Arduino Mega
  if (MegaSerial.available()) {
    String receivedData = MegaSerial.readStringUntil('\n');
    Serial.println("Received from Mega: " + receivedData);

    // Update the last received time
    lastReceivedTime = millis();

    // Parse and send data to Blynk
    if (receivedData.indexOf("ECG:") != -1) {
      int startIndex = 0;
      while (startIndex != -1) {
        startIndex = receivedData.indexOf("ECG:", startIndex);
        if (startIndex != -1) {
          int endIndex = receivedData.indexOf("ECG:", startIndex + 4);  // Look for next ECG value
          String ecg = (endIndex == -1) ? receivedData.substring(startIndex + 4) : receivedData.substring(startIndex + 4, endIndex);

          Serial.println("Parsed ECG Value: " + ecg);

          // Convert to integer and print continuously
          int ecgValue = ecg.toInt();
          Serial.println("ECG Value as Integer: " + String(ecgValue));

          Blynk.virtualWrite(V8, ecgValue);  // Send to Blynk
          delay(100);

          // Move to the next ECG value
          startIndex = endIndex;
        }
      }
    } else {

      Blynk.virtualWrite(V8, 0);
    }

    if (receivedData.indexOf("Body Temp:") != -1) {
      String bodyTemp = receivedData.substring(receivedData.indexOf("Body Temp:") + 10, receivedData.indexOf(","));
      Serial.println("Parsed Body Temp: " + bodyTemp);

      // Convert to double
      double bodyTempValue = bodyTemp.toFloat();  // Can be used as double
      Serial.println("Body Temp as double: " + String(bodyTempValue));

      Blynk.virtualWrite(V5, bodyTempValue);  // Send to Blynk app
    }

    if (receivedData.indexOf("Room Temp:") != -1) {
      String roomTemp = receivedData.substring(receivedData.indexOf("Room Temp:") + 10, receivedData.indexOf(", Humidity"));
      Serial.println("Parsed Room Temp: " + roomTemp);

      // Convert to double
      double roomTempValue = roomTemp.toFloat();  // Can be used as double
      Serial.println("Room Temp as double: " + String(roomTempValue));
      Blynk.virtualWrite(V6, roomTempValue);  // Send to Blynk app
    }

    if (receivedData.indexOf("Humidity:") != -1) {
      String humidity = receivedData.substring(receivedData.indexOf("Humidity:") + 9);
      Serial.println("Parsed Humidity: " + humidity);

      // Convert to double
      double humidityValue = humidity.toFloat();  // Can be used as double
      Serial.println("humidityValue as double: " + String(humidityValue));
      Blynk.virtualWrite(V7, humidityValue);  // Send to Blynk app
    }

    if (receivedData.indexOf("HeartRate:") != -1) {
      String heartRate = receivedData.substring(receivedData.indexOf("HeartRate:") + 10, receivedData.indexOf(",SpO2"));
      Serial.println("Parsed Heart Rate: " + heartRate);

      // Convert to integer
      int heartRateValue = heartRate.toInt();  // Converts string to integer
      Serial.println("Heart Rate as Integer: " + String(heartRateValue));

      // Send to Blynk app as an integer value
      Blynk.virtualWrite(V1, heartRateValue);
    }

    if (receivedData.indexOf("SpO2:") != -1) {
      String spo2 = receivedData.substring(receivedData.indexOf("SpO2:") + 5, receivedData.indexOf(",SystolicBP"));
      Serial.println("Parsed SpO2: " + spo2);

      // Convert to double
      double spo2Value = spo2.toFloat();  // Can be used as double
      Serial.println("spo2Value as double: " + String(spo2Value));
      Blynk.virtualWrite(V2, spo2Value);  // Send to Blynk app
    }

    if (receivedData.indexOf("SystolicBP:") != -1) {
      String systolicBP = receivedData.substring(receivedData.indexOf("SystolicBP:") + 11, receivedData.indexOf(",DiastolicBP"));
      Serial.println("Parsed Systolic BP: " + systolicBP);

      // Convert to double
      double systolicBPValue = systolicBP.toFloat();  // Can be used as double
      Serial.println("systolicBP as double: " + String(systolicBPValue));
      Blynk.virtualWrite(V3, systolicBPValue);  // Send to Blynk app
    }

    if (receivedData.indexOf("DiastolicBP:") != -1) {
      String diastolicBP = receivedData.substring(receivedData.indexOf("DiastolicBP:") + 12);
      Serial.println("Parsed Diastolic BP: " + diastolicBP);

      // Convert to double
      double diastolicBPValue = diastolicBP.toFloat();  // Can be used as double
      Serial.println("diastolicBPValue as double: " + String(diastolicBPValue));
      Blynk.virtualWrite(V4, diastolicBPValue);
    }
  }

  // Run Blynk's background tasks
  Blynk.run();

  // Check for no data received for 10 seconds
  if (millis() - lastReceivedTime > noDataInterval) {
    Serial.println("No data received from Arduino Mega in the last 10 seconds");
    lastReceivedTime = millis();  // Reset timer
  }

  delay(100);  // Minimal delay for stability
}
