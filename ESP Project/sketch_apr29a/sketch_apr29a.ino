#define PIN_SCK   4   // SPI Clock Pin
#define PIN_MOSI  6   // Master Out Slave In Pin
#define PIN_MISO  5   // Master In Slave Out Pin
#define PIN_CS    7   // Chip Select Pin

uint8_t recvBuf[25];         // Buffer to store 24 bytes + 1 CRC
const int numBytes = 25;     // 6 floats * 4 bytes + 1 CRC byte

// CRC-8 Calculator (Polynomial: 0x07)
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);

  // Start Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Setup complete. Waiting for SPI data...");
}

void loop() {
  // Wait for transaction start (CS pin goes low)
  Serial.println("Waiting for CS to go LOW...");
  while (digitalRead(PIN_CS) == HIGH) {
    delay(10);  // Small delay to prevent excessive CPU usage
  }
  Serial.println("CS is LOW. Start receiving data.");

  // Wait for SCK to go LOW before the first bit
  Serial.println("Waiting for SCK to go LOW...");
  while (digitalRead(PIN_SCK) == HIGH) {
    delay(10);  // Small delay to prevent excessive CPU usage
  }

  // Receive data byte by byte
  Serial.println("Receiving data...");
  for (int byteIndex = 0; byteIndex < numBytes; byteIndex++) {
    uint8_t receivedByte = 0;
    uint8_t toSend = 0xA5;  // Dummy byte to send back

    for (int i = 0; i < 8; i++) {
      // Wait for rising edge of SCK (start of bit transmission)
      while (digitalRead(PIN_SCK) == LOW);
      
      int bit = digitalRead(PIN_MOSI);
      receivedByte = (receivedByte << 1) | bit;
      
      // Send dummy bit to MISO
      digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
      toSend <<= 1;  // Shift toSend to prepare the next bit

      // Wait for falling edge of SCK (end of bit transmission)
      while (digitalRead(PIN_SCK) == HIGH);
    }

    recvBuf[byteIndex] = receivedByte;
    // Debugging received byte
    Serial.print("Byte ");
    Serial.print(byteIndex);
    Serial.print(": ");
    Serial.println(receivedByte, HEX);
  }

  // Calculate CRC over the received data (excluding the last byte, which is the CRC)
  uint8_t receivedCRC = recvBuf[numBytes - 1];
  uint8_t calculatedCRC = crc8(recvBuf, numBytes - 1);

  // Debugging CRC values
  Serial.print("Received CRC: ");
  Serial.println(receivedCRC, HEX);
  Serial.print("Calculated CRC: ");
  Serial.println(calculatedCRC, HEX);

  // Check if the calculated CRC matches the received CRC
  if (calculatedCRC == receivedCRC) {
    Serial.println("Valid CRC - Received Data:");

    // Extract the 6 float values from the buffer
    float Icmd1, Icmd2, velocity1, velocity2, theta1, theta2;

    memcpy(&Icmd1,     &recvBuf[0],  sizeof(float));
    memcpy(&Icmd2,     &recvBuf[4],  sizeof(float));
    memcpy(&velocity1, &recvBuf[8],  sizeof(float));
    memcpy(&velocity2, &recvBuf[12], sizeof(float));
    memcpy(&theta1,    &recvBuf[16], sizeof(float));
    memcpy(&theta2,    &recvBuf[20], sizeof(float));

    // Print the received float values
    Serial.print("Icmd1: "); Serial.println(Icmd1, 5);
    Serial.print("Icmd2: "); Serial.println(Icmd2, 5);
    Serial.print("Vel1 : "); Serial.println(velocity1, 5);
    Serial.print("Vel2 : "); Serial.println(velocity2, 5);
    Serial.print("Pos1 : "); Serial.println(theta1, 5);
    Serial.print("Pos2 : "); Serial.println(theta2, 5);
    Serial.println();
  } else {
    Serial.println("CRC mismatch! Data discarded.");
  }

  // Small delay before next loop iteration
  delay(2);
}
