//Constants
const int     MOTOR_SATURATION  = round(pow(2, PWM_RESOLUTION));
const float   BASE_WIDTH        = 0.1837;
const float   WHEEL_DIAMETER    = 0.0677;
const float   PULSES_PER_TURN   = 1320.0;
const float   BALANCE_POINT     = 0.05;
const float   SPEED_REFERENCE   = 0.0;
const float   DEADBAND_M1_POS   = 90.0;
const float   DEADBAND_M1_NEG   = 90.0;
const float   DEADBAND_M2_POS   = 90.0;
const float   DEADBAND_M2_NEG   = 90.0;


//Tuning
const float   K_SC              = 20.0;
const float   K_OL              = 13.0;
const float   K_IL              = 90.0;
const float   I_IL              = 5.5;
const float   filter_gain       = 15.0;


//Help variables
float         M1_Lin_Vel, M2_Lin_Vel;
float         M1_Ang_Vel, M2_Ang_Vel;
float         botLinVel , botAngVel ;
int           Speed_CMD, M1_Speed_CMD, M2_Speed_CMD;
float         ref_SC, act_SC, error_SC, SC_cont_out;
float         ref_OL, act_OL, error_OL, OL_cont_out;
float         ref_IL, act_IL, error_IL, iError_IL;




void initMotors() {
  //  float temp[] = {WHEEL_DIAMETER / 4, WHEEL_DIAMETER / 4, (WHEEL_DIAMETER / 2) / BASE_WIDTH, -(WHEEL_DIAMETER / 2) / BASE_WIDTH};
  //  int k = 0;
  //  for (int i = 0; i < 2; i++)
  //  {
  //    for (int j = 0; j < 2; j++)
  //    {
  //      inv_Kin[i][j] = temp[k];
  //      k = k + 1;
  //    }
  //  }

  inv_Kin[0][0] = WHEEL_DIAMETER / 4;
  inv_Kin[1][0] = (WHEEL_DIAMETER / 2) / BASE_WIDTH;
  inv_Kin[0][1] = WHEEL_DIAMETER / 4;
  inv_Kin[1][1] = -(WHEEL_DIAMETER / 2) / BASE_WIDTH;

  Matrix.Print((mtx_type*)inv_Kin, 2, 2, "Inverse kinematic matrix");
}

void motors() {


  // Speed Controller
  ref_SC          = SPEED_REFERENCE;
  act_SC          = vel_Matrix[0][0];
  error_SC        = ref_SC - act_SC;
  SC_cont_out     = error_SC * K_SC;


  // Balance controller
  // Outer loop
  ref_OL          = BALANCE_POINT - SC_cont_out;
  act_OL          = pitch;
  error_OL        = ref_OL - act_OL;
  OL_cont_out     = error_OL * K_OL;
  // Inner loop
  ref_IL          = OL_cont_out;
  act_IL          = pitch_rate;
  error_IL        = ref_IL - act_IL;
  iError_IL       = iError_IL + (error_IL * dT_s * I_IL);
  Speed_CMD       = round((error_IL * K_IL) + iError_IL);

  M1_Speed_CMD    = Speed_CMD;
  M2_Speed_CMD    = Speed_CMD;

  //  M1_Speed_CMD    = 500;
  //  M2_Speed_CMD    = 500;

  //Calculate speed from encoders
  M1_Lin_Vel          = encoderReaderLinVel(m1Raw, m1RawLast, M1_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);
  M2_Lin_Vel          = encoderReaderLinVel(m2Raw, m2RawLast, M2_Lin_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);
  M1_Ang_Vel          = encoderReaderAngVel(m1Raw, m1RawLast, M1_Ang_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);
  M2_Ang_Vel          = encoderReaderAngVel(m2Raw, m2RawLast, M2_Ang_Vel, PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);

  motor_ang_vel[0][0] = M1_Ang_Vel;
  motor_ang_vel[1][0] = M2_Ang_Vel;


  //void MatrixMath::Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
  //{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)


  Matrix.Multiply((mtx_type*)inv_Kin, (mtx_type*)motor_ang_vel, 2, 2, 1, (mtx_type*)vel_Matrix);


  //Motor control
  motorControl(1, M1_Speed_CMD, MOTOR_SATURATION, DEADBAND_M1_POS, DEADBAND_M1_NEG);
  motorControl(2, M2_Speed_CMD, MOTOR_SATURATION, DEADBAND_M2_POS, DEADBAND_M2_NEG);


  //  Serial plotter
  //  Serial.print("Balance_Point:");
  //  Serial.print(ref_OL);
  //  Serial.print(" ");
  //  Serial.print("Pitch_Angle:");
  //  Serial.print(act_OL);
  //  Serial.print(" ");
  //  Serial.print("Speed_CMD:");
  //  Serial.println(Speed_CMD * (100.0 / 4096.0));


  Serial.print("M1_Ang_Vel:");
  Serial.print(M1_Ang_Vel);
  Serial.print(" ");
  Serial.print("M2_Ang_Vel:");
  Serial.print(M2_Ang_Vel);
  Serial.print(" ");
  Serial.print("botLinVel:");
  Serial.print(vel_Matrix[0][0]);
  Serial.print(" ");
  Serial.print("botAngVel:");
  Serial.println(vel_Matrix[1][0]);


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  m2RawLast = m2Raw;

  //  Serial.print("m1Raw:");
  //  Serial.print(m1Raw);
  //  Serial.print(" ");
  //  Serial.print("m2Raw:");
  //  Serial.println(m2Raw);

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
  byte ch1 = motorID * 2 - 1;
  byte ch2 = motorID * 2;


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
