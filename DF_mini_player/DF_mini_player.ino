#include "DFRobotDFPlayerMini.h"

// Use hardware serial (Serial1) for DFPlayer Mini communication
#define FPSerial Serial1

DFRobotDFPlayerMini myDFPlayer;

void setup() {
  FPSerial.begin(9600);   // Initialize the serial communication with DFPlayer Mini
  Serial.begin(115200);   // Initialize the serial monitor

  // Initialize DFPlayer Mini
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {
    Serial.println(F("Unable to begin! Please check the connection and SD card."));
    while(true) { delay(0); }  // Halt the program if DFPlayer is not detected
  }
  
  Serial.println(F("DFPlayer Mini initialized."));
  myDFPlayer.volume(30);  // Set the volume (0 to 30)
}

void loop() {
  myDFPlayer.play(1);  // Play the first track (0001.mp3)
  delay(2000);         // Play for 1 second
  
  myDFPlayer.stop();   // Stop the music
  delay(2000);         // Wait for 2 seconds before playing again
}
