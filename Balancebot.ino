//Import
#include <GY_85.h>
#include <Wire.h>
#include <MatrixMath.h>
#include <Ps3Controller.h>
#include "arduino_secrets.h"

//Declare library objects
GY_85 IMU;


//GPIO PIN MAPPING
const byte M1_ENC_A = 32;
const byte M1_ENC_B = 33;
const byte M2_ENC_A = 34;
const byte M2_ENC_B = 35;
const byte M1_A = 16;
const byte M1_B = 17;
const byte M2_A = 18;
const byte M2_B = 19;
const int IMU_I2C_SDA = 26;
const int IMU_I2C_SCL = 27;


//Time variables
unsigned long tNow = micros();
unsigned long tLast = micros() + 13000;
int dT = 0;
float dT_s = 0.0;


//Encoder variables
long int m1Raw, m1RawLast;
long int m2Raw, m2RawLast;
volatile bool M1_A_state, M1_B_state;
volatile bool M2_A_state, M2_B_state;


//PS3 Controller variables
const char* _ps3Address = "18:5e:0f:92:00:6c";


//MotorControl Constants
const int PWM_CYCLE = 12000;
const byte PWM_RES = 12;
const int MOTOR_SATURATION = round(pow(2, PWM_RES));
const float BASE_WIDTH = 0.1837;
const float WHEEL_DIAMETER = 0.0677;
const float PULSES_PER_TURN = 1320.0;
const float BALANCE_POINT = 0.05;
const float SPEED_REF = 0.00;
const float TURN_SPEED_REF = 0.00;
const float DEADBAND_M1_POS = 90.0;
const float DEADBAND_M1_NEG = 90.0;
const float DEADBAND_M2_POS = 90.0;
const float DEADBAND_M2_NEG = 90.0;


//MotorControl Tuning
const float gainScale = 1;
const float K_SC = 18.5 * gainScale;  //Speed controller gain
const float K_TC = 90.0 * gainScale;  //Turn controller gain
const float K_OL = 13.0 * gainScale;  //Outer loop balance controller gain
const float K_IL = 72.0 * gainScale;  //Inner loop balance controller gain
const float I_IL = 80.0 * gainScale;  //Inner loop balance controller Igain
const float filter_gain = 16.0;       //Motor speed LPF gain


//MotorControl Help variables
int M1_Speed_CMD, M2_Speed_CMD;
float rem_speed_ref, rem_turn_speed_ref;
float SC_cont_out;
float TC_cont_out;
float OL_cont_out;
float ref_IL, act_IL, error_IL, IL_cont_out, iError_IL, IL_anti_windup;
float speedCmd1, speedCmd2;
bool balancingOn = false;


//MotorControl Matrices
mtx_type motor_ang_vel[2][1];
mtx_type vel_Matrix[2][1];
mtx_type inv_Kin[2][2];


//IMU CONSTANTS
const float alpha = 0.95;
const int acc_overflow_value = 65535;
const int gyro_overflow_value = 4558;  // 4096+512-50=4558 ?


//IMU VARIABLES
int ax, ay, az;
int cx, cy, cz;
int gx, gy, gz;
float gt;
float acc_pitch;
float pitch_rate;
float pitch = 0;
float pitch_prev = 0;


//UDP variables
uint8_t data[30 * 4];


void setup() {

  //Initialize serial
  Serial.begin(9600);
  delay(10);

  //Initialice I2C
  Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
  delay(10);

  //Initialize IMU
  IMU.init();
  delay(10);

  //Initialize encoder interrupts
  initEncoderInterrupt();

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

  ledcSetup(1, PWM_CYCLE, PWM_RES);
  ledcSetup(2, PWM_CYCLE, PWM_RES);
  ledcSetup(3, PWM_CYCLE, PWM_RES);
  ledcSetup(4, PWM_CYCLE, PWM_RES);

  //Initialize differential drive inverse kinematics
  initMotors();

  //Initialize PS3 controller connection
  Ps3.begin(_ps3Address);

  //Init UDP
  UdpInit();
}

void loop() {

  //Update time variables
  tNow = micros();
  dT = tNow - tLast;        //[Cycle time in microseconds]
  dT_s = dT * pow(10, -6);  //[Cycle time in seconds]

  //Get sensor data
  readIMU();

  //Control motors
  motors();

  // Plot
  SerialPlot();

  //Udp
  UdpLoop();

  //Save time for next cycle
  tLast = tNow;

  //Delay
  delay(5);
}
