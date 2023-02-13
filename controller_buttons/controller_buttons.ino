#include "Arduino.h"

// https://support.arduino.cc/hc/en-us/articles/360016724140-How-to-control-the-RGB-LED-and-Power-LED-of-the-Nano-33-BLE-boards-

#define RED 22     
#define BLUE 24     
#define GREEN 23
#define LED_PWR 25

const int button1Pin = D10;
const int button2Pin = D9;
int button1State = 0;
int button2State = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
}
void loop() {

  // Read buttons
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);

  Serial.println(String(button1State) + ":" + String(button2State));

  if (button1State == HIGH && button2State == HIGH) {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  } else if (button1State == HIGH) {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, HIGH);
  } else if (button2State == HIGH){
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  } else {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }
}

