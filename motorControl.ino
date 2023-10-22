//Constants
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


//Tuning
const float K_SC = 18.5*gainScale;         //Speed controller gain
const float K_TC = 90.0*gainScale;         //Turn controller gain
const float K_OL = 13.0*gainScale;         //Outer loop balance controller gain
const float K_IL = 72.0*gainScale;         //Inner loop balance controller gain
const float I_IL = 80.0*gainScale;         //Inner loop balance controller Igain
const float filter_gain = 16.0;  //Motor speed LPF gain
const float gainScale = 1;

//Help variables
int M1_Speed_CMD, M2_Speed_CMD;
float rem_speed_ref, rem_turn_speed_ref;
float SC_cont_out;
float TC_cont_out;
float OL_cont_out;
float ref_IL, act_IL, error_IL, IL_cont_out, iError_IL, IL_anti_windup;
float speedCmd1, speedCmd2;

bool balancingOn = false;

//Matrices
mtx_type motor_ang_vel[2][1];
mtx_type vel_Matrix[2][1];
mtx_type inv_Kin[2][2];


void initMotors() {
  // Inverse Kinematic matrix of differential drive robot
  inv_Kin[0][0] = WHEEL_DIAMETER / 4;
  inv_Kin[1][0] = (WHEEL_DIAMETER / 2) / BASE_WIDTH;
  inv_Kin[0][1] = WHEEL_DIAMETER / 4;
  inv_Kin[1][1] = -(WHEEL_DIAMETER / 2) / BASE_WIDTH;
}

void motors() {

  if (Ps3.data.button.cross) {
    ResetIntegrators();
    balancingOn = true;
  }

  if (Ps3.data.button.circle) {
    balancingOn = false;
  }

  if (Ps3.data.button.triangle) {
    ResetIntegrators();
  }

  if (Ps3.data.button.square) {
      IMU.init();
  }

  if (balancingOn) {

    //Calculate wheel angular velocity
    motor_ang_vel[0][0] = encoderReaderAngVel(m1Raw, m1RawLast, motor_ang_vel[0][0], PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);
    motor_ang_vel[1][0] = encoderReaderAngVel(m2Raw, m2RawLast, motor_ang_vel[1][0], PULSES_PER_TURN, WHEEL_DIAMETER, dT_s, filter_gain);


    //Calculate robot linear and angular velocity
    Matrix.Multiply((mtx_type*)inv_Kin, (mtx_type*)motor_ang_vel, 2, 2, 1, (mtx_type*)vel_Matrix);

    //Get Control Commands
    rem_turn_speed_ref = floatMap(Ps3.data.analog.stick.ly, -128.0, 127.0, -3.75, 3.75);
    rem_speed_ref = floatMap(Ps3.data.analog.stick.ry, -128.0, 127.0, -0.35, 0.35);

    // Speed Controller
    SC_cont_out = PController(rem_speed_ref, vel_Matrix[0][0], K_SC);


    // Balance controller
    // Outer loop
    OL_cont_out = PController((BALANCE_POINT - SC_cont_out), pitch, K_OL);
    // Inner loop
    ref_IL = OL_cont_out;
    act_IL = pitch_rate;
    error_IL = ref_IL - act_IL;
    iError_IL = iError_IL + (dT_s * (error_IL * I_IL) + (IL_anti_windup * ((1 / I_IL) + (1 / K_IL))));
    IL_cont_out = round((error_IL * K_IL) + iError_IL);


    //Turn controller
    TC_cont_out = PController(rem_turn_speed_ref, vel_Matrix[0][1], K_TC);

    //Sum speed command for motors
    M1_Speed_CMD = IL_cont_out - TC_cont_out;
    M2_Speed_CMD = IL_cont_out + TC_cont_out;

    //Motor control
    IL_anti_windup = motorControl(1, M1_Speed_CMD, MOTOR_SATURATION, DEADBAND_M1_POS, DEADBAND_M1_NEG);
    IL_anti_windup = IL_anti_windup + motorControl(2, M2_Speed_CMD, MOTOR_SATURATION, DEADBAND_M2_POS, DEADBAND_M2_NEG);
    IL_anti_windup = IL_anti_windup / 2;

  } else {

    //Sum speed command for motors
    speedCmd1 = floatMap(Ps3.data.analog.stick.ry, -128.0, 127.0, -1.0, 1.0);
    M1_Speed_CMD = MOTOR_SATURATION * speedCmd1;
    motorControl(1, M1_Speed_CMD, MOTOR_SATURATION, DEADBAND_M1_POS, DEADBAND_M1_NEG);

    speedCmd2 = floatMap(Ps3.data.analog.stick.ly, -128.0, 127.0, -1.0, 1.0);
    M2_Speed_CMD = MOTOR_SATURATION * speedCmd2;
    motorControl(2, M2_Speed_CMD, MOTOR_SATURATION, DEADBAND_M2_POS, DEADBAND_M2_NEG);
  }


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  m2RawLast = m2Raw;
}

void ResetIntegrators() {
  iError_IL = 0.0;
  IL_anti_windup = 0.0;
}

float PController(float ref_, float act_, float k_) {
  return (ref_ - act_) * k_;
}


float floatMap(int in, float inMin, float inMax, float outMin, float outMax) {
  return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float encoderReaderLinVel(int encRaw, int encRawLast, float lin_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, float dT_, float filt_gain_) {
  float dEnc_ = encRaw - encRawLast;        //[Number of encoder pulses this cycle]
  float dTurn_ = dEnc_ / pulses_per_turn_;  //[Amount wheel turned this cycle. 1 = full rotation]
  float lin_vel_ = (dTurn_ * wheel_diameter_ * PI) / (dT_);
  return lin_vel_filtered_ + ((lin_vel_ - lin_vel_filtered_) * dT_ * filt_gain_);
}

float encoderReaderAngVel(int encRaw, int encRawLast, float ang_vel_filtered_, float pulses_per_turn_, float wheel_diameter_, float dT_, float filt_gain_) {
  float dEnc_ = encRaw - encRawLast;        //[Number of encoder pulses this cycle]
  float dTurn_ = dEnc_ / pulses_per_turn_;  //[Amount wheel turned this cycle. 1 = full rotation]
  float ang_vel_ = (dTurn_ * 2 * PI) / (dT_);
  return ang_vel_filtered_ + ((ang_vel_ - ang_vel_filtered_) * dT_ * filt_gain_);
}

float motorControl(byte motorID, int speedCMD_, int saturation, float dbPos_, float dbNeg_) {
  //Returns anti windup difference
  //Calculate channel
  byte ch2 = motorID * 2;
  byte ch1 = ch2 - 1;
  float windup = 0;
  //Deadband
  if (speedCMD_ > 0 && speedCMD_ < dbPos_) {
    speedCMD_ = dbPos_;
  } else if (speedCMD_ < 0 && speedCMD_ > -dbNeg_) {
    speedCMD_ = -dbNeg_;
  }

  // Speed command saturation
  else if (speedCMD_ > saturation) {
    windup = saturation - speedCMD_;
    speedCMD_ = saturation;
  } else if (speedCMD_ < -saturation) {
    windup = saturation - speedCMD_;
    speedCMD_ = -saturation;
  }

  else {
    speedCMD_ = speedCMD_;
  }


  //Apply speed command to PWM output
  if (speedCMD_ > 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speedCMD_);
  } else if (speedCMD_ < 0) {
    ledcWrite(ch1, -1 * speedCMD_);
    ledcWrite(ch2, 0);
  } else if (speedCMD_ == 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
  }

  return windup;
}
