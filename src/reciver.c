#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct Data {
  int x;
  int y;
};

int motor1Pin1 = 3, motor1Pin2 = 5;
int motor2Pin1 = 6, motor2Pin2 = 9;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    Data data;
    radio.read(&data, sizeof(Data));

    int x = map(data.x, 0, 1023, -255, 255);
    int y = map(data.y, 0, 1023, -255, 255);

    if (y > 50) { 
      analogWrite(motor1Pin1, y);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, y);
      analogWrite(motor2Pin2, 0);
    } else if (y < -50) { 
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, -y);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, -y);
    } else if (x > 50) { 
      analogWrite(motor1Pin1, x);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, x);
    } else if (x < -50) { 
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, -x);
      analogWrite(motor2Pin1, -x);
      analogWrite(motor2Pin2, 0);
    } else { 
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, 0);
    }
  }
}
