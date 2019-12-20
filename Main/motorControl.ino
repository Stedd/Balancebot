//Constants
const int     MOTOR_SATURATION  = round(pow(2, PWM_RESOLUTION));
const float   WHEEL_DIAMETER    = 0.067708;
const float   PULSES_PER_TURN   = 1320.0;

//Tuning
const float   K                 = 3.5;
const float   I                 = 7.5;
const float   filter_gain       = 15;

//Help variables
float         M1_Lin_Vel, M2_Lin_Vel;
int           M1_Speed_CMD, M2_Speed_CMD;
float         M1_iError, M2_iError;

float         ref, act, error;

void motors() {

  //Controllers
  ref             = pitch * ((4096.0) / (90.0));
  act             = M1_Lin_Vel * 4096.0;
  error           = ref - act;
  M1_iError       = M1_iError + (error * dT * pow(10, -6) * I);
  M1_Speed_CMD    = round((error * K) + M1_iError);

//  ref             = pitch * ((4096.0) / (90.0));
//  act             = M2_Lin_Vel * 4096.0;
//  error           = ref - act;
//  M2_iError       = M2_iError + (error * dT * pow(10, -6) * I);
//  M2_Speed_CMD    = round((error * K) + M2_iError);




  //Calculate speed from encoders
  M1_Lin_Vel = encoderReader(m1Raw, m1RawLast, M1_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT, filter_gain);
  //  M2_Lin_Vel = encoderReader(m2Raw, m2RawLast, M2_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT, filter_gain);


  //Motor 1
  motorControl(1, M1_Speed_CMD, MOTOR_SATURATION);

  //Motor 2
//  motorSpeed(2, M2_Speed_CMD, MOTOR_SATURATION);
  motorControl(2, 0, MOTOR_SATURATION);


  //  Serial plotter
  Serial.print("M1_Speed_REF:");
  Serial.print(ref * (100.0 / 4096.0));
  Serial.print(" ");
  Serial.print("M1_Speed_ACT:");
  Serial.print(act * (100.0 / 4096.0));
  Serial.print(" ");
  Serial.print("M1_Speed_CMD:");
  Serial.println(M1_Speed_CMD * (100.0 / 4096.0));


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  m2RawLast = m2Raw;
}

float encoderReader(int encRaw, int encRawLast, float lin_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, int dT_, float filt_gain_ ) {
  float dEnc_    = encRaw - encRawLast;        //[Number of encoder pulses this cycle]
  float dTurn_   = dEnc_ / pulses_per_turn_;   //[Amount wheel turned this cycle. 1 = full rotation]
  float lin_vel_ = (dTurn_ * wheel_diameter_ * PI) / (dT_ * 0.000001);
  return lin_vel_filtered_ + ((lin_vel_ - lin_vel_filtered_) * dT_ * 0.000001 * filt_gain_);
}

void motorControl(byte motorID, int speedCMD_, int saturation) {
  //Calculate channel
  byte ch1 = motorID * 2 - 1;
  byte ch2 = motorID * 2;


  // Speed command saturation
  if (speedCMD_ > saturation) {
    speedCMD_ = saturation;
  }
  else if (speedCMD_ < -saturation) {
    speedCMD_ = -saturation;
  }
  else {
    speedCMD_ = speedCMD_;
  }


  //Motor Control
  if (speedCMD_ > 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speedCMD_);
  }
  else if (speedCMD_ < 0) {
    ledcWrite(ch1, -1 * speedCMD_);
    ledcWrite(ch2, 0);
  }
}
