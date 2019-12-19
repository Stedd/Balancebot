//Import
#include "GY_85.h"
#include <Wire.h>

GY_85 GY85;     //create the object

//System variables
unsigned long tNow = micros();

//Motor Control variables
const byte m1_in1 = 18;
const byte m1_in2 = 19;
//int m2_in1 = 16;
//int m2_in2 = 17;

//Encoders variables
long int m1Raw,m1RawLast;
const byte pin_m1_A = 34;
const byte pin_m1_B = 35;
volatile bool A_state;
volatile bool B_state;

//Interrupt routines
//  CW =  INCREASE
//  CCW = DECREASE

void IRAM_ATTR m1_A_changed() {
  A_state = digitalRead(pin_m1_A);
  B_state = digitalRead(pin_m1_B);
  
  //Rising
  if (A_state == HIGH) {
    if (B_state == HIGH) {
      m1Raw = m1Raw - 1;
    }
    else if (B_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }
  
  //Falling
  else if (A_state == LOW) {
    if (B_state == HIGH) {
      m1Raw = m1Raw + 1;
    }
    else if (B_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }
}

void IRAM_ATTR m1_B_changed() {
  A_state = digitalRead(pin_m1_A);
  B_state = digitalRead(pin_m1_B);
  
  //Rising
  if (B_state == HIGH) {
    if (A_state == HIGH) {
      m1Raw = m1Raw + 1;
    }
    else if (A_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }

  //Falling
  else if (B_state == LOW) {
    if (A_state == HIGH) {
      m1Raw = m1Raw - 1;
    }
    else if (A_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }
}


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

  //Initialize Interrupts
  pinMode(pin_m1_A, INPUT_PULLUP);
  pinMode(pin_m1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_m1_A), m1_A_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_m1_B), m1_B_changed, CHANGE);

  //Initialize Encoders
  m1Raw = 0;
  m1RawLast = 100;

  // Initialize PWM channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);

  ledcAttachPin(m1_in1, 1);
  ledcAttachPin(m1_in2, 2);
  //  ledcAttachPin(m2_in1, 3);
  //  ledcAttachPin(m2_in2, 4);

  ledcSetup(1, 12000, 12); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 12);
  //  ledcSetup(3, 12000, 8);
  //  ledcSetup(4, 12000, 8);

Serial.println("Reference,Actual,SpeedCommand");

}

void loop() {
  //Update system variables
  tNow = micros();

////  //Only print encoder value if value changed since last print
//  if (m1Raw != m1RawLast) {
//    Serial.println(m1Raw);
//    m1RawLast = m1Raw;
//  }

  //Sense
  readIMU();


  //Think


  //Act
  motorControl();


  //Delay
  delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
