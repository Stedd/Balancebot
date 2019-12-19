void motorControl() {
  //Motor 1
  speed_setp = round (map(acc_pitch, -90.0, 90.0, -4096, 4096));
  if (speed_setp > 0) {
    ledcWrite(1, speed_setp);
    ledcWrite(2, 0);
  }
  else if (speed_setp < 0) {
    ledcWrite(1, 0);
    ledcWrite(2, -1 * speed_setp);
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
}
