#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[20];  // Enough to store "Hello ESP" + null

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  // Wait for CS LOW (start of transaction)
  while (digitalRead(PIN_CS) == HIGH);

  // Wait for SCK to go LOW before first bit
  while (digitalRead(PIN_SCK) == HIGH);

  const int numBytes = 9; // Expecting "Hello ESP"
  for (int byteIndex = 0; byteIndex < numBytes; byteIndex++) {
    uint8_t receivedByte = 0;
    uint8_t toSend = 0xA5; // Dummy byte to send back

    for (int i = 0; i < 8; i++) {
      // Wait for rising edge of SCK
      while (digitalRead(PIN_SCK) == LOW);

      // Sample MOSI
      int bit = digitalRead(PIN_MOSI);
      receivedByte = (receivedByte << 1) | bit;

      // Write dummy bit to MISO
      digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
      toSend <<= 1;

      // Wait for falling edge
      while (digitalRead(PIN_SCK) == HIGH);
    }

    recvBuf[byteIndex] = receivedByte;
  }

  recvBuf[numBytes] = '\0'; // Null terminate string

  Serial.print("Received: ");
  Serial.println((char*)recvBuf); // Should print: Hello ESP
}
