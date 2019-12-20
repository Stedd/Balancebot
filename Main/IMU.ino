//IMU variables
int   ax_, ay_, az_;
int   ax, ay, az;
int   cx, cy, cz;
float gx, gy, gz, gt;
float acc_pitch;

void readIMU() {
  ax_ = IMU.accelerometer_x( IMU.readFromAccelerometer() );
  ay_ = IMU.accelerometer_y( IMU.readFromAccelerometer() );
  az_ = IMU.accelerometer_z( IMU.readFromAccelerometer() );

  //  cx = IMU.compass_x( IMU.readFromCompass() );
  //  cy = IMU.compass_y( IMU.readFromCompass() );
  //  cz = IMU.compass_z( IMU.readFromCompass() );

  gx = IMU.gyro_x( IMU.readGyro() );
  gy = IMU.gyro_y( IMU.readGyro() );
  gz = IMU.gyro_z( IMU.readGyro() );
  //  gt = IMU.temp  ( IMU.readGyro() );


  //For some reason the ints in the example behaves as unsigned int.. Maybe look at the .cpp code, might be something there, if not. This works OK.
  if (ax_ > 60000) {
    ax = (ax_ - 65535);
  }
  else  {
    ax = ax_;
  }

  if (ay_ > 60000) {
    ay = (ay_ - 65535);
  }
  else  {
    ay = ay_;
  }

  if (az_ > 60000) {
    az = (az_ - 65535);
  }
  else  {
    az = az_;
  }

  acc_pitch = -1*atan2(ax,sqrt((pow(ay,2)+pow(az,2))))*180/3.14;
  

//Serial plotter
//  map(ax,-140,140,-1,1)
//  map(ay,-140,140,-1,1)
//  map(az,-140,140,-1,1)

//      Serial.print  ( " x:" );
//  Serial.print  ( ax );
//  Serial.print  ( "," );
//  Serial.print  ( ay );
//  Serial.print  ( "," );
//  Serial.println  ( az );
//  Serial.print  ( "," );
//  Serial.println  ( acc_pitch);



}
