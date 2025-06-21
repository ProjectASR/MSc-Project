#include <Adafruit_NeoPixel.h>

// Bit-banged SPI pins
#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[24];  // 6 floats x 4 bytes each = 24 bytes

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  // Wait for SPI transaction
  while (digitalRead(PIN_CS) == HIGH);
  while (digitalRead(PIN_SCK) == HIGH);

  // Receive 24 bytes
  for (int byteIndex = 0; byteIndex < 24; byteIndex++) {
    uint8_t receivedByte = 0;
    uint8_t toSend = 0xA5;

    for (int i = 0; i < 8; i++) {
      while (digitalRead(PIN_SCK) == LOW);
      int bit = digitalRead(PIN_MOSI);
      receivedByte = (receivedByte << 1) | bit;

      digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
      toSend <<= 1;
      while (digitalRead(PIN_SCK) == HIGH);
    }

    recvBuf[byteIndex] = receivedByte;
  }

  // Check if received data is all zeros
  bool allZero = true;
  for (int i = 0; i < 24; i++) {
    if (recvBuf[i] != 0) {
      allZero = false;
      break;
    }
  }

  if (!allZero) {
    float desired_torque, filtered_reaction_torque, SDCardCount, torque_error, theta1, theta2;
    memcpy(&desired_torque, &recvBuf[0], sizeof(float));
    memcpy(&filtered_reaction_torque, &recvBuf[4], sizeof(float));
    memcpy(&SDCardCount, &recvBuf[8], sizeof(float));
    memcpy(&torque_error, &recvBuf[12], sizeof(float));
    memcpy(&theta1, &recvBuf[16], sizeof(float));
    memcpy(&theta2, &recvBuf[20], sizeof(float));

    Serial.print("desired_torque:");
    Serial.print(desired_torque);
    Serial.print("\t");

    Serial.print("filtered_reaction_torque:");
    Serial.print(filtered_reaction_torque);
    Serial.print("\t");

    Serial.print("theta1:");
    Serial.print(theta1);
    Serial.print("\t");

    Serial.print("theta2:");
    Serial.print(theta2);
    Serial.println();
  }
}
