//Constants
const int     MOTOR_SATURATION  = round(pow(2, PWM_RES));
const float   BASE_WIDTH        = 0.1837;
const float   WHEEL_DIAMETER    = 0.0677;
const float   PULSES_PER_TURN   = 1320.0;
const float   BALANCE_POINT     = 0.05;
const float   SPEED_REF         = 0.00;
const float   TURN_SPEED_REF    = 0.00;
const float   DEADBAND_M1_POS   = 90.0;
const float   DEADBAND_M1_NEG   = 90.0;
const float   DEADBAND_M2_POS   = 90.0;
const float   DEADBAND_M2_NEG   = 90.0;


//Tuning
const float   K_SC              = 18.0;   //Speed controller gain
const float   K_TC              = 130.0;  //Turn controller gain
const float   K_OL              = 14.0;   //Outer loop balance controller gain
const float   K_IL              = 85.0;   //Inner loop balance controller gain
const float   I_IL              = 5.25;   //Inner loop balance controller Igain
const float   filter_gain       = 16.0;   //Motor speed LPF gain


//Help variables
int           M1_Speed_CMD, M2_Speed_CMD;
float         rem_speed_ref, rem_turn_speed_ref;
float         ref_SC, act_SC, error_SC, SC_cont_out;
float         ref_TC, act_TC, error_TC, TC_cont_out;
float         ref_OL, act_OL, error_OL, OL_cont_out;
float         ref_IL, act_IL, error_IL, IL_cont_out, iError_IL;


//Matrices
mtx_type      motor_ang_vel   [2][1];
mtx_type      vel_Matrix      [2][1];
mtx_type      inv_Kin         [2][2];


void initMotors() {
  // Inverse Kinematic matrix of differential drive robot
  inv_Kin[0][0]       = WHEEL_DIAMETER / 4;
  inv_Kin[1][0]       = (WHEEL_DIAMETER / 2) / BASE_WIDTH;
  inv_Kin[0][1]       = WHEEL_DIAMETER / 4;
  inv_Kin[1][1]       = -(WHEEL_DIAMETER / 2) / BASE_WIDTH;
}

void motors() {


  //Calculate wheel angular velocity
  motor_ang_vel[0][0] = encoderReaderAngVel(m1Raw, m1RawLast, motor_ang_vel[1][0], PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);
  motor_ang_vel[1][0] = encoderReaderAngVel(m2Raw, m2RawLast, motor_ang_vel[1][0], PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);


  //Calculate robot linear and angular velocity
  Matrix.Multiply((mtx_type*)inv_Kin, (mtx_type*)motor_ang_vel, 2, 2, 1, (mtx_type*)vel_Matrix);


  // Remote control commands
  rem_turn_speed_ref  = floatMap(pwm_time_ch1, 992.0, 2007.0, -3.5, 3.5);
  rem_speed_ref       = floatMap(pwm_time_ch2, 982.0, 1997.0, -0.25, 0.25);


  // Speed Controller
  SC_cont_out         = PController(rem_speed_ref, vel_Matrix[0][0], K_SC);


  // Balance controller
  // Outer loop
  OL_cont_out         = PController((BALANCE_POINT - SC_cont_out), pitch, K_OL);
  // Inner loop
  ref_IL              = OL_cont_out;
  act_IL              = pitch_rate;
  error_IL            = ref_IL - act_IL;
  iError_IL           = iError_IL + (error_IL * dT_s * I_IL);
  IL_cont_out         = round((error_IL * K_IL) + iError_IL);


  //Turn controller
  TC_cont_out         = PController(rem_turn_speed_ref, vel_Matrix[0][1], K_TC);


  //Sum speed command for motors
  M1_Speed_CMD        = IL_cont_out - TC_cont_out;
  M2_Speed_CMD        = IL_cont_out + TC_cont_out;

  //Sum speed command for motors
  // M1_Speed_CMD    = 0;
  // M2_Speed_CMD    = 0;


  //Motor control
  motorControl(1, M1_Speed_CMD, MOTOR_SATURATION, DEADBAND_M1_POS, DEADBAND_M1_NEG);
  motorControl(2, M2_Speed_CMD, MOTOR_SATURATION, DEADBAND_M2_POS, DEADBAND_M2_NEG);


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  m2RawLast = m2Raw;


}

float PController(float ref_, float act_, float k_){
  return (ref_-act_)*k_;
}


float floatMap(int in, float inMin, float inMax, float outMin, float outMax){
  return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float encoderReaderLinVel(int encRaw, int encRawLast, float lin_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, float dT_, float filt_gain_ ) {
  float dEnc_     = encRaw - encRawLast;                                  //[Number of encoder pulses this cycle]
  float dTurn_    = dEnc_ / pulses_per_turn_;                             //[Amount wheel turned this cycle. 1 = full rotation]
  float lin_vel_  = (dTurn_ * wheel_diameter_ * PI) / (dT_);
  return          lin_vel_filtered_ + ((lin_vel_ - lin_vel_filtered_) * dT_ * filt_gain_);
}

float encoderReaderAngVel(int encRaw, int encRawLast, float ang_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, float dT_, float filt_gain_ ) {
  float dEnc_     = encRaw - encRawLast;                                  //[Number of encoder pulses this cycle]
  float dTurn_    = dEnc_ / pulses_per_turn_;                             //[Amount wheel turned this cycle. 1 = full rotation]
  float ang_vel_  = (dTurn_ * 2 * PI) / (dT_);
  return          ang_vel_filtered_ + ((ang_vel_ - ang_vel_filtered_) * dT_ * filt_gain_);
}

void motorControl(byte motorID, int speedCMD_, int saturation, float dbPos_, float dbNeg_) {
  //Calculate channel
  byte ch2 = motorID * 2;
  byte ch1 = ch2 - 1;

  //Deadband
  if (speedCMD_ > 0 && speedCMD_ < dbPos_) {
    speedCMD_ = dbPos_;
  }
  else if (speedCMD_ < 0 && speedCMD_ > -dbNeg_) {
    speedCMD_ = -dbNeg_;
  }

  // Speed command saturation
  else if (speedCMD_ > saturation) {
    speedCMD_ = saturation;
  }
  else if (speedCMD_ < -saturation) {
    speedCMD_ = -saturation;
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
  else if (speedCMD_ == 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
  }
}
