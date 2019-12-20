//Import
#include <GY_85.h>
#include <Wire.h>


//Declare library objects
GY_85 IMU;


//GPIO PIN MAPPING
const byte    M1_ENC_A      = 34;
const byte    M1_ENC_B      = 35;
const byte    M2_ENC_A      = 32;
const byte    M2_ENC_B      = 33;
const byte    M1_A          = 18;
const byte    M1_B          = 19;
const byte    M2_A          = 16;
const byte    M2_B          = 17;
const byte    IMU_I2C_SCL   = 26;
const byte    IMU_I2C_SDA   = 27;


//Time variables
unsigned long tNow          = micros();
unsigned long tLast         = micros() + 13000;
int           dT            = 0;


//Motor variables
const int   PWM_CYCLE       = 12000;
const byte  PWM_RESOLUTION  = 12;

//Encoders variables
long int        m1Raw, m1RawLast;
long int        m2Raw, m2RawLast;
volatile bool   M1_A_state;
volatile bool   M1_B_state;
volatile bool   M2_A_state;
volatile bool   M2_B_state;


//Interrupt routines
//  CW =  INCREASE
//  CCW = DECREASE


void IRAM_ATTR m1_A_changed() {
  M1_A_state = digitalRead(M1_ENC_A);
  M1_B_state = digitalRead(M1_ENC_B);

  //Rising
  if (M1_A_state == HIGH) {
    if (M1_B_state == HIGH) {
      m1Raw = m1Raw - 1;
    }
    else if (M1_B_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }

  //Falling
  else if (M1_A_state == LOW) {
    if (M1_B_state == HIGH) {
      m1Raw = m1Raw + 1;
    }
    else if (M1_B_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }
}


void IRAM_ATTR m1_B_changed() {
  M1_A_state = digitalRead(M1_ENC_A);
  M1_B_state = digitalRead(M1_ENC_B);

  //Rising
  if (M1_B_state == HIGH) {
    if (M1_A_state == HIGH) {
      m1Raw = m1Raw + 1;
    }
    else if (M1_A_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }

  //Falling
  else if (M1_B_state == LOW) {
    if (M1_A_state == HIGH) {
      m1Raw = m1Raw - 1;
    }
    else if (M1_A_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }
}

void IRAM_ATTR m2_A_changed() {
  M2_A_state = digitalRead(M2_ENC_A);
  M2_B_state = digitalRead(M2_ENC_B);

  //Rising
  if (M2_A_state == HIGH) {
    if (M2_B_state == HIGH) {
      m2Raw = m2Raw - 1;
    }
    else if (M2_B_state == LOW) {
      m2Raw = m2Raw + 1;
    }
  }

  //Falling
  else if (M2_A_state == LOW) {
    if (M2_B_state == HIGH) {
      m2Raw = m2Raw + 1;
    }
    else if (M2_B_state == LOW) {
      m2Raw = m2Raw - 1;
    }
  }
}


void IRAM_ATTR m2_B_changed() {
  M2_A_state = digitalRead(M2_ENC_A);
  M2_B_state = digitalRead(M2_ENC_B);

  //Rising
  if (M2_B_state == HIGH) {
    if (M2_A_state == HIGH) {
      m2Raw = m2Raw + 1;
    }
    else if (M2_A_state == LOW) {
      m2Raw = m2Raw - 1;
    }
  }

  //Falling
  else if (M2_B_state == LOW) {
    if (M2_A_state == HIGH) {
      m2Raw = m2Raw - 1;
    }
    else if (M2_A_state == LOW) {
      m2Raw = m2Raw + 1;
    }
  }
}


void setup() {
  //Initialize serial
  Serial.begin(57600);
  delay(10);

  //Initialice I2C
  Wire.begin(IMU_I2C_SCL, IMU_I2C_SDA);
  delay(10);

  //Initialize IMU
  IMU.init();
  //Might need some logic here to mke sure the gyro is calibrated correctly, or hardcode the values...
  IMU.GyroCalibrate();
  delay(10);

  //Initialize encoder interrupts
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), m1_A_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), m1_B_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), m2_A_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), m2_B_changed, CHANGE);

  //Initialize encoders
  m1Raw = 0;
  m1RawLast = 100;
  m2Raw = 0;
  m2RawLast = 100;

  // Initialize PWM channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);

  ledcAttachPin(M1_A, 1);
  ledcAttachPin(M1_B, 2);
  ledcAttachPin(M2_A, 3);
  ledcAttachPin(M2_B, 4);

  ledcSetup(1, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(2, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(3, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(4, PWM_CYCLE, PWM_RESOLUTION);

  //  Serial.println("Reference,Actual,SpeedCommand");

}

void loop() {
  //Update time variables
  tNow  = micros();
  dT    = tNow - tLast;             //[Cycle time in microseconds]


  //Get sensor data
  readIMU();


  //Control motor
  motors();


  //Save time
  tLast = tNow;


  //Delay
  delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
