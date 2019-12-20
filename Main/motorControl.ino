//Constants
const float   wheel_diameter  = 0.067708;
const float   pulses_per_turn = 1320.0;
const float   filt_gain = 15;
//Tuning
const float   K = 3.5;
const float   I = 7.5;

//Help variables
int           dEnc;
float         dTurn, ang_vel, lin_vel, lin_vel_prev, ang_vel_filtered, lin_vel_filtered;
float         dVel, diff;

float         speed_setp, referenceSpeed, actualSpeed, actualSpeedFiltered;
float         error, iError;
int           speedCMD;


void motorControl() {

  //Speed Controller
  referenceSpeed  = pitch * ((4096.0) / (90.0));
  actualSpeed     = lin_vel * 4096.0;
  actualSpeedFiltered = lin_vel_filtered * 4096.0;
  error           = referenceSpeed - actualSpeedFiltered;
  iError          = iError + (error * dT * pow(10, -6) * I);
  speedCMD        = round((error * K) + iError);


  // Speed command saturation
  if (speedCMD > 4096) {
    speedCMD = 4096;
  }
  else if (speedCMD < -4096) {
    speedCMD = -4096;
  }
  else {
    speedCMD = speedCMD;
  }


  //Motor 1 Control
  if (speedCMD > 0) {
    ledcWrite(1, 0);
    ledcWrite(2, speedCMD);
  }
  else if (speedCMD < 0) {
    ledcWrite(1, -1 * speedCMD);
    ledcWrite(2, 0);
  }



  //Motor 2
  //  ledcWrite(3, 255);
  //  ledcWrite(4, 255);


  //No speed command
  //  ledcWrite(1, 0);
  //  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(4, 0);


  //Calculate speed from encoders
  dEnc    = m1Raw - m1RawLast;        //[Number of encoder pulses this cycle]

  dTurn   = dEnc / pulses_per_turn;   //[Amount wheel turned this cycle. 1 = full rotation]

  ang_vel = (dTurn * 2 * PI) / (dT * 0.000001);
  lin_vel = (dTurn * wheel_diameter * PI) / (dT * 0.000001);

  //  Lowpass filter
  //  lin_vel_filtered        = lin_vel_filtered + ((lin_vel - lin_vel_filtered)*(1-abs(diff))* dT * pow(10, -6)*filt_gain);
  lin_vel_filtered        = lin_vel_filtered + ((lin_vel - lin_vel_filtered) * dT * pow(10, -6) * filt_gain);


  //  Serial plotter
  Serial.print("referenceSpeed:");
  Serial.print(referenceSpeed * (100.0 / 4096.0));
  Serial.print(" ");
//  Serial.print("actualSpeed:");
//  Serial.print(actualSpeed * (100.0 / 4096.0));
//  Serial.print(" ");
  Serial.print("actualSpeedFiltered:");
  Serial.print(actualSpeedFiltered * (100.0 / 4096.0));
  Serial.print(" ");
  Serial.print("speedCMD:");
  Serial.println(speedCMD * (100.0 / 4096.0));
  //    Serial.print  ( "," );
  //  Serial.println(lin_vel);
  //  Serial.print  ( "," );


  //Update variables for next scan cycle
  m1RawLast = m1Raw;


}
