//Import
#include "GY_85.h"
#include <Wire.h>

//Define L298N pin mappings
const int IN1 = 3;
const int IN2 = 2;
const int IN3 = 5;
const int IN4 = 4;
#define LOW 0

GY_85 GY85;     //create the object

int   ax_, ay_, az_;
int   ax, ay, az;
int   cx, cy, cz;
float gx, gy, gz, gt;



void setup() {

  //  Wire.begin(SCL,SDA);
  Wire.begin(26, 27);
  delay(10);
  Serial.begin(57600);
  delay(10);
  GY85.init();
  delay(10);

  //Assign H bridge Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void loop() {
  readIMU();
  motorControl();

  //Delay
  delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
