#include <SPI.h>

// Define custom SPI pins
#define SCK_PIN 4
#define MISO_PIN 5
#define MOSI_PIN 6
#define SS_PIN 7  // Chip Select (CS) Pin

void setup() {
  // Start the Serial Monitor for debugging
  Serial.begin(115200);
  while (!Serial);  // Wait for the Serial Monitor to be ready

  // Initialize SPI for ESP32 (Slave mode) with custom pins
  pinMode(SS_PIN, INPUT_PULLUP);  // Set CS pin as input

  // Initialize the SPI interface with custom pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  // Set SPI parameters (same as STM32 configuration)
  SPI.beginTransaction(SPISettings(2250000, MSBFIRST, SPI_MODE0));  // 500kHz, MSB first, SPI Mode 0

  Serial.println("ESP32 SPI Slave Initialized");
}

void loop() {
  // Check if Chip Select (CS) pin is active (low)
  if (digitalRead(SS_PIN) == LOW) {
    // Receive data via SPI
    byte incomingByte = SPI.transfer(0);  // Receive data (sending 0 as no data to send back)

    // Print the received data to Serial Monitor
    Serial.print("Received Data: ");
    Serial.println(incomingByte, DEC);  // Print received byte as decimal

    // Optionally, you could send a response back to the master if needed
    // byte responseByte = 0x01;  // Example response
    // SPI.transfer(responseByte);  // Send response back
  }
}
