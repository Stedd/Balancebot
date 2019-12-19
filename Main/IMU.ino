void readIMU() {
  ax_ = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay_ = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az_ = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  //  cx = GY85.compass_x( GY85.readFromCompass() );
  //  cy = GY85.compass_y( GY85.readFromCompass() );
  //  cz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );
  //  gt = GY85.temp  ( GY85.readGyro() );


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

  //    Serial.print  ( " x:" );
//  Serial.print  ( ax );
//  Serial.print  ( "," );
//  Serial.print  ( ay );
//  Serial.print  ( "," );
//  Serial.print  ( az );
//  Serial.print  ( "," );
//  Serial.println  ( acc_pitch);



}
