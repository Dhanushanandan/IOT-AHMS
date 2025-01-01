#include <SoftwareSerial.h>

// Define TX and RX pins for SoftwareSerial
#define NODEMCU_TX D1  // GPIO5
#define NODEMCU_RX D2  // GPIO4

// Create a SoftwareSerial object
SoftwareSerial MegaSerial(NODEMCU_RX, NODEMCU_TX);  // RX, TX

unsigned long lastReceivedTime = 0;         // To track the last data received time
const unsigned long noDataInterval = 10000;  // Time interval to check for no data (10 seconds)

void setup() {
  Serial.begin(9600);      // Debugging (Hardware Serial)
  MegaSerial.begin(9600);  // Communication with Arduino Mega (Software Serial)

  Serial.println("NodeMCU ready!");
}

void loop() {
  // Check if data is available from Arduino Mega
  if (MegaSerial.available()) {
    String receivedData = MegaSerial.readStringUntil('\n');
    Serial.println("Received from Mega: " + receivedData);

    // Update the last received time
    lastReceivedTime = millis();

    // Parse and log data based on its type
    if (receivedData.indexOf("Body Temp:") != -1) {
      String bodyTemp = receivedData.substring(receivedData.indexOf("Body Temp:") + 10, receivedData.indexOf(","));
      Serial.println("Body Temperature: " + bodyTemp);
    }
    if (receivedData.indexOf("Room Temp:") != -1) {
      String roomTemp = receivedData.substring(receivedData.indexOf("Room Temp:") + 10, receivedData.indexOf(", Humidity"));
      Serial.println("Room Temperature: " + roomTemp);
    }
    if (receivedData.indexOf("Humidity:") != -1) {
      String humidity = receivedData.substring(receivedData.indexOf("Humidity:") + 9);
      Serial.println("Humidity: " + humidity);
    }

    // Parsing other sensor data
    if (receivedData.indexOf("HeartRate:") != -1) {
      String heartRate = receivedData.substring(receivedData.indexOf("HeartRate:") + 10, receivedData.indexOf(",SpO2"));
      Serial.println("Heart Rate: " + heartRate);
    }
    if (receivedData.indexOf("SpO2:") != -1) {
      String spo2 = receivedData.substring(receivedData.indexOf("SpO2:") + 5, receivedData.indexOf(",SystolicBP"));
      Serial.println("SpO2: " + spo2);
    }
    if (receivedData.indexOf("SystolicBP:") != -1) {
      String systolicBP = receivedData.substring(receivedData.indexOf("SystolicBP:") + 11, receivedData.indexOf(",DiastolicBP"));
      Serial.println("Systolic BP: " + systolicBP);
    }
    if (receivedData.indexOf("DiastolicBP:") != -1) {
      // Parsing diastolic BP after the systolic BP value
      String diastolicBP = receivedData.substring(receivedData.indexOf("DiastolicBP:") + 12);
      Serial.println("Diastolic BP: " + diastolicBP);
    }
  }

  // Check if no data has been received for a defined interval
  if (millis() - lastReceivedTime > noDataInterval) {
    Serial.println("No data received from Arduino Mega in the last 10 seconds");
    lastReceivedTime = millis();  // Reset the timer to avoid repeated prints
  }

  delay(100);  // Minimal delay for stability
}
