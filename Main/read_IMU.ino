void readIMU()
{
  ax_ = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay_ = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az_ = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  cx = GY85.compass_x( GY85.readFromCompass() );
  cy = GY85.compass_y( GY85.readFromCompass() );
  cz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );
  gt = GY85.temp  ( GY85.readGyro() );

  //    Serial.print  ( "accelerometer" );
  //    Serial.print  ( " x:" );
  //    Serial.print  ( ax );
  //    Serial.print  ( " y:" );
  //    Serial.print  ( ay );
  //    Serial.print  ( " z:" );
  //    Serial.print  ( az );

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


  //Serial plotter
  //    Serial.print  ( " x:" );
  Serial.print  ( ax );
  Serial.print  ( "," );
  Serial.print  ( ay );
  Serial.print  ( "," );
  Serial.println  ( az );

  //        Serial.print(deg_R);
  //    Serial.print(",");
  //    Serial.println(deg_L);

  //    Serial.print  ( "  compass" );
  //    Serial.print  ( " x:" );
  //    Serial.print  ( cx );
  //    Serial.print  ( " y:" );
  //    Serial.print  ( cy );
  //    Serial.print  (" z:");
  //    Serial.print  ( cz );

  //    Serial.print  ( "  gyro" );
  //    Serial.print  ( " x:" );
  //    Serial.print  ( gx );
  //    Serial.print  ( " y:" );
  //    Serial.print  ( gy );
  //    Serial.print  ( " z:" );
  //    Serial.print  ( gz );
  //    Serial.print  ( " gyro temp:" );
  //    Serial.println( gt );


}
