//CONSTANTS
const float   alpha               = 0.95;
const int     acc_overflow_value  = 65535;
const int     gyro_overflow_value = 4558;   // 4096+512-50=4558 ?


//IMU VARIABLES
int           ax_, ay_, az_;
int           ax, ay, az;
int           cx_, cy_, cz_;
int           gx_, gy_, gz_;
int           gx, gy, gz;
//float gt;
float         acc_pitch;
float         pitch_rate;
float         dAngle, estAngle;
float         pitch               = 0;
float         pitch_prev          = 0;


void readIMU() {
  ax_ = IMU.accelerometer_x( IMU.readFromAccelerometer() );
  ay_ = IMU.accelerometer_y( IMU.readFromAccelerometer() );
  az_ = IMU.accelerometer_z( IMU.readFromAccelerometer() );

  //  cx = IMU.compass_x( IMU.readFromCompass() );
  //  cy = IMU.compass_y( IMU.readFromCompass() );
  //  cz = IMU.compass_z( IMU.readFromCompass() );

  gx_ = IMU.gyro_x( IMU.readGyro() );
  gy_ = IMU.gyro_y( IMU.readGyro() );
  gz_ = IMU.gyro_z( IMU.readGyro() );
  //  gt = IMU.temp  ( IMU.readGyro() );


  //Convert accelerometer
  ax = convertInt(ax_, acc_overflow_value);
  ay = convertInt(ay_, acc_overflow_value);
  az = convertInt(az_, acc_overflow_value);


  //Convert gyro
  //  Gyroscope coordinate system
  //  gx -  Pitch rate
  //  gy -  Roll rate
  //  gz -  Yaw rate
  //  Gyro is calibrated for +-2000deg/s
  //  Conversion is happening in GY_85.h line 48-50
  gx = convertInt(gx_, gyro_overflow_value);
  gy = convertInt(gy_, gyro_overflow_value);
  gz = convertInt(gz_, gyro_overflow_value);


  // Pitch angle from accelerometer
  acc_pitch = -1 * atan2(ax, sqrt((pow(ay, 2) + pow(az, 2)))) * 180.0 / PI;


  //Pitch rate from gyroscope
  pitch_rate = -gx;


  //Complementary filter
  dAngle      = pitch_rate * dT * pow(10, -6);
  pitch       = acc_pitch * (1 - alpha) + (dAngle + pitch_prev * alpha);
  pitch_prev  = pitch;


  //Serial plotter
  //  Serial.print  ( "Pitch:" );
  //  Serial.print  ( pitch );
  //  Serial.print  (" ");
  //  Serial.print  ( "Accelerometer_Pitch:" );
  //  Serial.print  ( acc_pitch );
  //  Serial.print  (" ");
  //  Serial.print  ( "," );
  //  Serial.println  ( gz );
  //  Serial.print  ( "," );
  //  Serial.println  ( gt );
  //  Serial.print  ( "," );
  //  Serial.println  ( acc_pitch);
}


int convertInt(int raw, int overflow_value_) {
  //For some reason the ints in the example behaves as unsigned int.. Maybe look at the .cpp code, might be something there, if not. This works OK.
  int conv_val;

  if (raw > (overflow_value_ / 2)) {
    conv_val = (raw - overflow_value_);
  }

  else  {
    conv_val = raw;
  }

  return conv_val;

}
