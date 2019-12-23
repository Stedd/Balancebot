//CONSTANTS
const float   alpha               = 0.95;
const int     acc_overflow_value  = 65535;
const int     gyro_overflow_value = 4558;   // 4096+512-50=4558 ?


//IMU VARIABLES
int           ax, ay, az;
int           cx, cy, cz;
int           gx, gy, gz;
float         gt;
float         acc_pitch;
float         pitch_rate;
float         dAngle, estAngle;
float         pitch               = 0;
float         pitch_prev          = 0;


void readIMU() {
  //Acceletometer
  ax  = convertInt(IMU.accelerometer_x( IMU.readFromAccelerometer() ), acc_overflow_value);
  ay  = convertInt(IMU.accelerometer_y( IMU.readFromAccelerometer() ), acc_overflow_value);
  az  = convertInt(IMU.accelerometer_z( IMU.readFromAccelerometer() ), acc_overflow_value);


  //Magnetometer
  cx = IMU.compass_x( IMU.readFromCompass() );
  cy = IMU.compass_y( IMU.readFromCompass() );
  cz = IMU.compass_z( IMU.readFromCompass() );


  //  Gyrocope
  gx  = convertInt(IMU.gyro_x( IMU.readGyro() ), gyro_overflow_value);    //  gx -  Pitch rate
  gy  = convertInt(IMU.gyro_y( IMU.readGyro() ), gyro_overflow_value);    //  gy -  Roll rate
  gz  = convertInt(IMU.gyro_z( IMU.readGyro() ), gyro_overflow_value);    //  gz -  Yaw rate


  //Temperature sensor
  gt = IMU.temp  ( IMU.readGyro() );


  // Pitch angle from accelerometer
  acc_pitch = -1 * atan2(ax, sqrt((pow(ay, 2) + pow(az, 2)))) * 180.0 / PI;


  //Pitch rate from gyroscope
  pitch_rate = -gx;


  //Complementary filter
  dAngle      = pitch_rate * dT_s;
  pitch       = acc_pitch * (1 - alpha) + ((dAngle + pitch_prev) * alpha);
  pitch_prev  = pitch;

  
}


int convertInt(int raw, int overflow_value_) {
  //For some reason the ints in the example behaves as unsigned int.. Maybe look at the .cpp code, might be something there, if not. This works OK.

  if (raw > (overflow_value_ / 2)) {
    return (raw - overflow_value_);
  }

  else  {
    return raw;
  }
}
