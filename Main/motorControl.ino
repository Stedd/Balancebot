//Constants
const float   wheel_diameter  = 0.067708;
const float   pulses_per_turn = 1320.0;
//Tuning
const float   K = 2.0;
const float   I = 10.0;

//Help variables
int           dEnc, dT;
float         dTurn, ang_vel, lin_vel, ang_vel_filtered, lin_vel_filtered;
unsigned long tLast;

float         speed_setp, referenceSpeed, actualSpeed;
float         error, iError;
int           speedCMD;


void motorControl() {

  //Speed Controller
  referenceSpeed  = acc_pitch * ((4096.0) / (90.0));
  actualSpeed     = lin_vel * 4096.0;
  error           = referenceSpeed - actualSpeed;
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
  //  Serial.print  (acc_pitch);
  //  Serial.print  ( "," );
  //  Serial.println(speed_setp);

  //
  //  ledcWrite(1, 0);
  //  ledcWrite(2, 0);


  //Motor 2
  //  ledcWrite(3, 255);
  //  ledcWrite(4, 255);


  //Calculate speed from encoders
  dEnc    = m1Raw - m1RawLast;        //[Number of encoder pulses this cycle]
  dT      = tNow - tLast;             //[Cycle time in microseconds]
  dTurn   = dEnc / pulses_per_turn;   //[Amount wheel turned this cycle. 1 = full rotation]

  ang_vel = (dTurn * 2 * PI) / (dT * 0.000001);
  lin_vel = (dTurn * wheel_diameter * PI) / (dT * 0.000001);

  Serial.print(referenceSpeed);
  Serial.print  ( "," );
  Serial.print(actualSpeed);
  Serial.print  ( "," );
  Serial.println(speedCMD);
  //  Serial.print  ( "," );
  //  Serial.println(lin_vel);
  //  Serial.print  ( "," );


  //Update variables for next scan cycle
  m1RawLast = m1Raw;
  tLast = tNow;

}
