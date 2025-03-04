#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct Data {
  int x;
  int y;
};

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  Data data;
  data.x = analogRead(A0); // Joystick X
  data.y = analogRead(A1); // Joystick Y
  radio.write(&data, sizeof(Data));
  delay(50);
}
