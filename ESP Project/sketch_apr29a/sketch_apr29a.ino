#include <Adafruit_NeoPixel.h>

// NeoPixel configuration
#define NEOPIXEL_PIN 8
#define NUMPIXELS    1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Bit-banged SPI pins
#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[24];  // 6 floats x 4 bytes each = 24 bytes

void setup() {
  // NeoPixel setup
  pixels.begin();
  pixels.setBrightness(50);

  // Bit-banged SPI pin setup
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);

  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Bit-banged SPI slave read
  while (digitalRead(PIN_CS) == HIGH);  // Wait for transaction start
  while (digitalRead(PIN_SCK) == HIGH); // Wait for SCK LOW before bit start

  // Receive 24 bytes (6 floats)
  for (int byteIndex = 0; byteIndex < 24; byteIndex++) {
    uint8_t receivedByte = 0;
    uint8_t toSend = 0xA5;

    for (int i = 0; i < 8; i++) {
      while (digitalRead(PIN_SCK) == LOW);  // Wait for rising edge of SCK
      int bit = digitalRead(PIN_MOSI);
      receivedByte = (receivedByte << 1) | bit;

      // Send dummy byte to MISO
      digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
      toSend <<= 1;

      while (digitalRead(PIN_SCK) == HIGH);  // Wait for falling edge of SCK
    }

    recvBuf[byteIndex] = receivedByte;
  }

  // Convert the byte buffer to float values
  float desired_torque, filtered_reaction_torque, SDCardCount, torque_error, theta1, theta2;
  memcpy(&desired_torque, &recvBuf[0], sizeof(float));
  memcpy(&filtered_reaction_torque, &recvBuf[4], sizeof(float));
  memcpy(&SDCardCount, &recvBuf[8], sizeof(float));
  memcpy(&torque_error, &recvBuf[12], sizeof(float));
  memcpy(&theta1, &recvBuf[16], sizeof(float));
  memcpy(&theta2, &recvBuf[20], sizeof(float));

  // Print received values to Serial Plotter
  Serial.print("desired_torque:");
  Serial.print(desired_torque);
  Serial.print("\t");

  Serial.print("filtered_reaction_torque:");
  Serial.print(filtered_reaction_torque);
  Serial.print("\t");
  Serial.println("");
  if(desired_torque-filtered_reaction_torque < 0.2){
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
  pixels.show();
  }
  else{
  // Flash RGB LED after reception
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
  pixels.show();
 }
}
