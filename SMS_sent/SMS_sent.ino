#include <SoftwareSerial.h>

// Define SIM800L pins
#define RX_PIN 2
#define TX_PIN 3

SoftwareSerial sim800l(RX_PIN, TX_PIN);

void setup() {
  Serial.begin(9600); // Begin Serial Monitor communication
  sim800l.begin(9600); // Begin SIM800L communication

  Serial.println("Initializing SIM800L...");
  delay(5000); // Wait for the module to initialize

  // Step-by-step initialization and diagnostics
  if (sendCommand("AT", "OK", 2000)) {
    Serial.println("SIM800L is communicating.");
  } else {
    Serial.println("Error: SIM800L not responding.");
    return;
  }

  if (sendCommand("AT+CPIN?", "+CPIN: READY", 5000)) {
    Serial.println("SIM card is ready.");
  } else {
    Serial.println("Error: SIM card not detected or locked.");
    return;
  }

  if (sendCommand("AT+CSQ", "OK", 2000)) {
    Serial.println("Signal strength checked.");
  } else {
    Serial.println("Error: Signal strength check failed.");
  }

  if (sendCommand("AT+CREG?", "0,1", 5000) || sendCommand("AT+CREG?", "0,5", 5000)) {
    Serial.println("SIM800L is registered to the network.");
  } else {
    Serial.println("Error: Network registration failed.");
    return;
  }

  if (sendCommand("AT+CMGF=1", "OK", 2000)) {
    Serial.println("SMS mode set to text.");
  } else {
    Serial.println("Error: Failed to set SMS text mode.");
  }
}

void loop() {
  // Send an SMS periodically
  if (sendSMS("+94752051204", "Hello from SIM800L!")) {
    Serial.println("SMS sent successfully.");
  } else {
    Serial.println("Failed to send SMS.");
  }
  delay(60000); // Wait 1 minute before sending another SMS
}

// Function to send a command and check for the expected response
bool sendCommand(String command, String expectedResponse, unsigned long timeout) {
  Serial.println("Sending: " + command);
  sim800l.println(command);

  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    if (sim800l.available()) {
      char c = sim800l.read();
      response += c;
      Serial.print(c);

      if (response.indexOf(expectedResponse) != -1) {
        Serial.println("\nResponse received: " + response);
        return true;
      }
    }
  }

  Serial.println("\nTimeout waiting for response: " + expectedResponse);
  Serial.println("Full response: " + response);
  return false;
}

// Function to send an SMS
bool sendSMS(String phoneNumber, String message) {
  Serial.println("Sending SMS...");

  sim800l.print("AT+CMGS=\"");
  sim800l.print(phoneNumber);
  sim800l.println("\"");
  delay(1000);

  if (waitForPrompt(">", 5000)) {
    sim800l.print(message);
    sim800l.write(26); // Send Ctrl+Z
    Serial.println("\nMessage content sent: " + message);

    if (waitForResponse("OK", 10000)) {
      Serial.println("SMS sent successfully.");
      return true;
    } else {
      Serial.println("Error: SMS send failed.");
    }
  } else {
    Serial.println("Error: No '>' prompt received.");
  }

  return false;
}

// Function to wait for a specific prompt
bool waitForPrompt(String prompt, unsigned long timeout) {
  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    if (sim800l.available()) {
      char c = sim800l.read();
      response += c;
      Serial.print(c);

      if (response.indexOf(prompt) != -1) {
        Serial.println("\nPrompt received: " + prompt);
        return true;
      }
    }
  }

  Serial.println("\nTimeout waiting for prompt: " + prompt);
  return false;
}

// Function to wait for a specific response
bool waitForResponse(String expectedResponse, unsigned long timeout) {
  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    if (sim800l.available()) {
      char c = sim800l.read();
      response += c;
      Serial.print(c);

      if (response.indexOf(expectedResponse) != -1) {
        Serial.println("\nExpected response received.");
        return true;
      }
    }
  }

  Serial.println("\nTimeout waiting for response: " + expectedResponse);
  Serial.println("Full response: " + response);
  return false;
}