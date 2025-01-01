#include <SoftwareSerial.h>

// Define TX and RX pins for SoftwareSerial
#define NODEMCU_TX D1 // GPIO5
#define NODEMCU_RX D2 // GPIO4

// Create a SoftwareSerial object
SoftwareSerial MegaSerial(NODEMCU_RX, NODEMCU_TX); // RX, TX

unsigned long lastReceivedTime = 0; // To track the last data received time
const unsigned long noDataInterval = 5000; // Time interval to check for no data (5 seconds)

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

    // Respond back to Arduino Mega
    MegaSerial.println("Ack: " + receivedData);
  }

  // Check if no data has been received for a defined interval
  if (millis() - lastReceivedTime > noDataInterval) {
    Serial.println("No data received from Arduino Mega in the last 5 seconds");
    lastReceivedTime = millis(); // Reset the timer to avoid repeated prints
  }

  delay(100); // Minimal delay for stability
}
