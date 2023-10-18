//CONSTANTS
const float alpha = 0.95;
const int acc_overflow_value = 65535;
const int gyro_overflow_value = 4558;  // 4096+512-50=4558 ?


//IMU VARIABLES
int ax, ay, az;
int cx, cy, cz;
float gx, gy, gz;
float gt;
float acc_pitch;
float pitch_rate;
float pitch = 0;
float pitch_prev = 0;


void readIMU() {
  // Serial.println("ReadingIMU");
  //Acceletometer
  int* accelerometerReadings = IMU.readFromAccelerometer();
  ax = convertInt(IMU.accelerometer_x(accelerometerReadings), acc_overflow_value);
  ay = convertInt(IMU.accelerometer_y(accelerometerReadings), acc_overflow_value);
  az = convertInt(IMU.accelerometer_z(accelerometerReadings), acc_overflow_value);


  //Magnetometer
  int* compassReadings = IMU.readFromCompass();
  cx = IMU.compass_x(compassReadings);
  cy = IMU.compass_y(compassReadings);
  cz = IMU.compass_z(compassReadings);


  //  Gyrocope
  float* gyroReadings = IMU.readGyro();
  gx = convertInt(IMU.gyro_x(gyroReadings), gyro_overflow_value);  //  gx -  Pitch rate
  gy = convertInt(IMU.gyro_y(gyroReadings), gyro_overflow_value);  //  gy -  Roll rate
  gz = convertInt(IMU.gyro_z(gyroReadings), gyro_overflow_value);  //  gz -  Yaw rate


  //Temperature sensor
  gt = IMU.temp(gyroReadings);


  // Pitch angle from accelerometer
  acc_pitch = -1 * atan2(ax, sqrt((pow(ay, 2) + pow(az, 2)))) * 180.0 / PI;


  //Pitch rate from gyroscope
  pitch_rate = -gx;


  //Complementary filter
  pitch = acc_pitch * (1 - alpha) + (((pitch_rate * dT_s) + pitch_prev) * alpha);
  pitch_prev = pitch;
}


int convertInt(int raw, int overflow_value_) {
  //For some reason the ints in the example behaves as unsigned int.. Maybe look at the .cpp code, might be something there, if not. This works OK.

  if (raw > (overflow_value_ / 2)) {
    return (raw - overflow_value_);
  }

  else {
    return raw;
  }
}
