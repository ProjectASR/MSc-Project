#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[24];  // 6 floats = 24 bytes

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);

  digitalWrite(PIN_MISO, LOW);  // Ensure line is LOW by default
  Serial.begin(115200);
}

// Robust SPI byte reader with timeout (prevents lockup)
uint8_t ReadByteSPI(unsigned long timeout_us = 2000) {
  uint8_t receivedByte = 0;
  uint8_t toSend = 0xA5;

  for (int i = 0; i < 8; i++) {
    unsigned long startTime = micros();
    while (digitalRead(PIN_SCK) == LOW) {
      if (micros() - startTime > timeout_us) return 0xFF; // timeout
    }

    delayMicroseconds(1);  // MOSI stabilizing time
    int bit = digitalRead(PIN_MOSI);
    receivedByte = (receivedByte << 1) | (bit & 0x01);

    digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
    toSend <<= 1;

    startTime = micros();
    while (digitalRead(PIN_SCK) == HIGH) {
      if (micros() - startTime > timeout_us) return 0xFF;
    }
  }

  return receivedByte;
}

bool WaitForCSLow(unsigned long timeout_ms = 100) {
  unsigned long start = millis();
  while (digitalRead(PIN_CS) == HIGH) {
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

bool WaitForCSHigh(unsigned long timeout_ms = 100) {
  unsigned long start = millis();
  while (digitalRead(PIN_CS) == LOW) {
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

void loop() {
  if (!WaitForCSLow()) return;

  // Wait for 0xAA start byte
  uint8_t startByte = 0;
  do {
    startByte = ReadByteSPI();
  } while (startByte != 0xAA && digitalRead(PIN_CS) == LOW);

  if (startByte != 0xAA || digitalRead(PIN_CS) == HIGH) return;

  // Read remaining 24 bytes
  for (int i = 0; i < 24; i++) {
    uint8_t byte = ReadByteSPI();
    if (byte == 0xFF && digitalRead(PIN_CS) == LOW) {
      //Serial.println("Timeout on byte read.");
      return;
    }
    recvBuf[i] = byte;
  }

  // Wait for CS to go HIGH
  if (!WaitForCSHigh()) return;

  // Extract float values
  float filtered_reaction_torque, desired_torque, velocity1, velocity2, theta1, theta2;
  uint32_t MainTaskCount;
  memcpy(&MainTaskCount, &recvBuf[0], sizeof(uint32_t));
  memcpy(&desired_torque,        &recvBuf[4],  sizeof(float));
  memcpy(&filtered_reaction_torque,             &recvBuf[8],  sizeof(float));
  memcpy(&velocity2,             &recvBuf[12], sizeof(float));
  memcpy(&theta1,                &recvBuf[16], sizeof(float));
  memcpy(&theta2,                &recvBuf[20], sizeof(float));

  // Debug output
  Serial.print("MainTaskCount: "); Serial.print(MainTaskCount); Serial.print("\t");
  Serial.print("desired_torque: ");        Serial.print(desired_torque, 4);        Serial.print("\t");
  Serial.print("filtered_reaction_torque: ");             Serial.print(filtered_reaction_torque, 4);             Serial.print("\t");
  Serial.print("velocity2: ");             Serial.print(velocity2, 4);             Serial.print("\t");
  Serial.print("theta1: ");                Serial.print(theta1, 4);                Serial.print("\t");
  Serial.print("theta2: ");                Serial.println(theta2, 4);

  memset(recvBuf, 0, sizeof(recvBuf));
  delay(5);
}
