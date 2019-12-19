//Import
#include "GY_85.h"
#include <Wire.h>


GY_85 GY85;     //create the object

int   ax_, ay_, az_;
int   ax, ay, az;
int   cx, cy, cz;
float gx, gy, gz, gt;
float acc_pitch;
signed int speed_setp;

int i, modifier;

int m1_in1 = 33;
int m1_in2 = 25;
int m2_in1 = 13;
int m2_in2 = 14;

void setup() {
  //Initialize serial
  Serial.begin(57600);
  delay(10);

  //Initialice I2C
  //  Wire.begin(SCL,SDA);
  Wire.begin(26, 27);
  delay(10);

  //Initialize IMU
  GY85.init();
  delay(10);

  // Initialize PWM channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);

  ledcAttachPin(m1_in1, 1);
  ledcAttachPin(m1_in2, 2);
  ledcAttachPin(m2_in1, 3);
  ledcAttachPin(m2_in2, 4);

  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);
  ledcSetup(3, 12000, 8);
  ledcSetup(4, 12000, 8);

  i = 0;
  modifier = 2;
}

void loop() {
  //Sense
  readIMU();


  //Think


  //Act
  motorControl();

  i=i+modifier;

    if (i>=4096){
      modifier = -2;
    }
    else if (i<=0){
      modifier = 2;
    }

  //Delay
  delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
