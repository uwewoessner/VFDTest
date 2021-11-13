
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>


#include <ESP32Encoder.h>

ESP32Encoder encoder1;

#define LED_ON LOW
#define LED_OFF HIGH

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN 22 // Pin D2 mapped to pin GPI22/ADC12 of ESP32, control on-board LED


#define DAC1 25

void setup()
{
  //set led pin as output
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(32,INPUT_PULLUP);
  pinMode(33,INPUT_PULLUP);
  pinMode(26,OUTPUT);
  pinMode(25,OUTPUT);
  
  
  //digitalWrite(LED_BUILTIN,LED_ON);
  Serial.begin(115200);
  while (!Serial);

  delay(200);
  //digitalWrite(LED_BUILTIN,LED_ON);

	ESP32Encoder::useInternalWeakPullResistors=UP;

	encoder1.attachHalfQuad(32, 33);
  encoder1.setCount(0);

  Serial.println("\nStarting \n"); 


}


void toggleLED()
{
  //toggle state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void loop()
{
  
  float degree = fabs((encoder1.getCount()%1200)/1200.0 * 360.0);
  Serial.println(degree); 
  //Serial.println(digitalRead(33));
  //Serial.println(digitalRead(32)); 
  
  dacWrite(DAC1, 255*(degree/360.0));
  
  delay(200);
}
