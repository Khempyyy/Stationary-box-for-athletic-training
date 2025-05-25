#include "Wire.h"
volatile boolean Start_BUTTON_triggered = false;
#define Start_BUTTON_pin 7
#define Start_BUTTON_INTpin 8

#define red_LED_pin 0

void setup() {
  pinMode(red_LED_pin,OUTPUT);
  pinMode(Start_BUTTON_pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Start_BUTTON_INTpin), interrupt1, CHANGE);
}

void loop() {
  Serial.println(Start_BUTTON_triggered);
}

void interrupt1()
{
  if(Start_BUTTON_triggered)
  {
    Start_BUTTON_triggered = !Start_BUTTON_triggered;
    digitalWrite(red_LED_pin,LOW);
    Serial.println("off");
  }else
  {
    Start_BUTTON_triggered = !Start_BUTTON_triggered;
    digitalWrite(red_LED_pin,HIGH);
    Serial.println("on");
  }
}
