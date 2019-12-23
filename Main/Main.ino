//Import
#include <GY_85.h>
#include <Wire.h>
#include <MatrixMath.h>


//Declare library objects
GY_85         IMU;


//GPIO PIN MAPPING
const byte    M1_ENC_A      = 32;
const byte    M1_ENC_B      = 33;
const byte    M2_ENC_A      = 34;
const byte    M2_ENC_B      = 35;
const byte    M1_A          = 16;
const byte    M1_B          = 17;
const byte    M2_A          = 18;
const byte    M2_B          = 19;
const byte    IMU_I2C_SCL   = 26;
const byte    IMU_I2C_SDA   = 27;


//Time variables
unsigned long tNow          = micros();
unsigned long tLast         = micros() + 13000;
int           dT            = 0;
float         dT_s          = 0.0;


//Motor variables
const int   PWM_CYCLE       = 12000;
const byte  PWM_RESOLUTION  = 12;


//Encoders variables
long int        m1Raw, m1RawLast;
long int        m2Raw, m2RawLast;
volatile bool   M1_A_state, M1_B_state;
volatile bool   M2_A_state, M2_B_state;


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
  initInterrupt();


  //Initialize encoders
  m1Raw = 0;
  m1RawLast = 100;
  m2Raw = 0;
  m2RawLast = 100;


  // Initialize PWM channels
  ledcAttachPin(M1_A, 1);
  ledcAttachPin(M1_B, 2);
  ledcAttachPin(M2_A, 3);
  ledcAttachPin(M2_B, 4);

  ledcSetup(1, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(2, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(3, PWM_CYCLE, PWM_RESOLUTION);
  ledcSetup(4, PWM_CYCLE, PWM_RESOLUTION);


  //Initialize differential drive inverse kinematics
  initMotors();


  // Initialize Remote control
  initRemote();

}

void loop() {
  //Update time variables
  tNow  = micros();
  dT    = tNow - tLast;             //[Cycle time in microseconds]
  dT_s  = dT * pow(10,-6);          //[Cycle time in seconds]


  //Get sensor data
  readIMU();


  //Get remote control data
  readRemote();


  //Control motors
  motors();


  //Save time for next cycle
  tLast = tNow;


  // Plot
  plot();


  //Delay
  delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
