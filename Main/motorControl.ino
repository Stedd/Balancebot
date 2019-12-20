//Constants
const int     MOTOR_SATURATION  = round(pow(2, PWM_RESOLUTION));
const float   WHEEL_DIAMETER    = 0.067708;
const float   PULSES_PER_TURN   = 1320.0;
const float   BALANCE_POINT     = -0.05;
const float   SPEED_REFERENCE   = 0.0;
const float   DEADBAND_M1_POS   = 90.0;
const float   DEADBAND_M1_NEG   = 90.0;
const float   DEADBAND_M2_POS   = 90.0;
const float   DEADBAND_M2_NEG   = 90.0;

//Tuning
const float   K_SC              = 15.0;
const float   K_OL              = 13.0;
const float   K_IL              = 80.0;
const float   I_IL              = 5.5;
const float   filter_gain       = 15.0;

//Help variables
float         M1_Lin_Vel, M2_Lin_Vel;
int           Speed_CMD, M1_Speed_CMD, M2_Speed_CMD;

float         ref_SC, act_SC, error_SC, SC_cont_out;
float         ref_OL, act_OL, error_OL, OL_cont_out;
float         ref_IL, act_IL, error_IL, iError_IL;

void motors() {


  // Speed Controller
  ref_SC          = SPEED_REFERENCE;
  act_SC          = (M1_Lin_Vel + M2_Lin_Vel) / 2;
  error_SC        = ref_SC - act_SC;
  SC_cont_out     = (error_SC * K_SC);


  // Balance controller
  // Outer loop
  ref_OL          = BALANCE_POINT - SC_cont_out;
  act_OL          = pitch;
  error_OL        = ref_OL - act_OL;
  OL_cont_out     = (error_OL * K_OL);
  // Inner loop
  ref_IL          = OL_cont_out;
  act_IL          = pitch_rate;
  error_IL        = ref_IL - act_IL;
  iError_IL       = iError_IL + (error_IL * dT * pow(10, -6) * I_IL);
  Speed_CMD       = round((error_IL * K_IL) + iError_IL);

  M1_Speed_CMD    = Speed_CMD;
  M2_Speed_CMD    = Speed_CMD;

//  M1_Speed_CMD    = 0;
//  M2_Speed_CMD    = 0;

  //Calculate speed from encoders
  M1_Lin_Vel = encoderReader(m1Raw, m1RawLast, M1_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT, filter_gain);
  M2_Lin_Vel = encoderReader(m2Raw, m2RawLast, M2_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT, filter_gain);


  //Motor control
  motorControl(1, M1_Speed_CMD, MOTOR_SATURATION, DEADBAND_M1_POS, DEADBAND_M1_NEG);
  motorControl(2, M2_Speed_CMD, MOTOR_SATURATION, DEADBAND_M2_POS, DEADBAND_M2_NEG);


  //  Serial plotter
  Serial.print("Balance_Point:");
  Serial.print(ref_OL);
  Serial.print(" ");
  Serial.print("Pitch_Angle:");
  Serial.print(act_OL);
  Serial.print(" ");
  Serial.print("Speed_CMD:");
  Serial.println(Speed_CMD * (100.0 / 4096.0));


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  m2RawLast = m2Raw;
}

float encoderReader(int encRaw, int encRawLast, float lin_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, int dT_, float filt_gain_ ) {
  float dEnc_     = encRaw - encRawLast;        //[Number of encoder pulses this cycle]
  float dTurn_    = dEnc_ / pulses_per_turn_;   //[Amount wheel turned this cycle. 1 = full rotation]
  float lin_vel_  = (dTurn_ * wheel_diameter_ * PI) / (dT_ * 0.000001);
  return          lin_vel_filtered_ + ((lin_vel_ - lin_vel_filtered_) * dT_ * 0.000001 * filt_gain_);
}

void motorControl(byte motorID, int speedCMD_, int saturation, float dbPos_, float dbNeg_) {
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

  
  //Deadband
  else if (speedCMD_ > 0 && speedCMD_ < dbPos_) {
    speedCMD_ = dbPos_;
  }
  else if (speedCMD_ < 0 && speedCMD_ > -dbNeg_) {
    speedCMD_ = -dbNeg_;
  }

  //Zero speed if input = 0
  else if (speedCMD_ == 0) {
    speedCMD_ = 0;
  }
  else {
    speedCMD_ = speedCMD_;
  }


  //Apply speed command to PWM output
  if (speedCMD_ > 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speedCMD_);
  }
  else if (speedCMD_ < 0) {
    ledcWrite(ch1, -1 * speedCMD_);
    ledcWrite(ch2, 0);
  }
}
