void plot() {
  // Time
  // Serial.print("dT:");
  // Serial.println(dT);
  // Serial.print(" ");
  // Serial.print("dT_s:");
  // Serial.println(dT_s);
  // Serial.print(" ");

  // IMU
  Serial.print  ( "Pitch:" );
  Serial.println  ( pitch );
  // Serial.print  (" ");
  // Serial.print  ( "Accelerometer_Pitch:" );
  // Serial.print  ( acc_pitch );
  // Serial.print  (" ");
  // Serial.print  ( "," );
  // Serial.println  ( gz );
  // Serial.print  ( "," );
  // Serial.println  ( gt );
  // Serial.print  ( " " );
  // Serial.println  ( acc_pitch);


  // Remote control
  // Serial.print("ch1:");
  // Serial.print(pwm_time_ch1);
  // Serial.print(" ");
  // Serial.print("ch2:");
  // Serial.print(pwm_time_ch2);
  // Serial.print("ch1mapped:");
  // Serial.print(rem_turn_speed_ref);
  // Serial.print(" ");
  // Serial.print("ch2mapped:");
  // Serial.println(rem_speed_ref);


  // Encoders
  // Serial.print("m1Raw:");
  // Serial.print(m1Raw);
  // Serial.print(" ");
  // Serial.print("m2Raw:");
  // Serial.println(m2Raw);

  // Motors
  // Serial.print("SpeedControllerOut:");
  // Serial.print(SC_cont_out);
  // Serial.print(" ");
  // Serial.print("BalanceOLControllerOut:");
  // Serial.print(OL_cont_out);
  // Serial.print(" ");
  // Serial.print("BalanceILControllerOut:");
  // Serial.print(IL_cont_out);
  // Serial.print(" ");
  // Serial.print("SpeedCmd1:");
  // Serial.println(speedCmd1);
  // Serial.print(" ");
  // Serial.print("M1_CMD:");
  // Serial.print(M1_Speed_CMD);
  // Serial.print(" ");
  // Serial.print("SpeedCmd2:");
  // Serial.println(speedCmd2);
  // Serial.print(" ");
  // Serial.print("M2_CMD:");
  // Serial.println(M2_Speed_CMD);

  // Serial.print("M1_Ang_Vel:");
  // Serial.print(motor_ang_vel[0][0]);
  // Serial.print(" ");
  // Serial.print("M2_Ang_Vel:");
  // Serial.print(motor_ang_vel[0][1]);
  // Serial.print(" ");
  // Serial.print("botLinVel:");
  // Serial.print(vel_Matrix[0][0]);
  // Serial.print(" ");
  // Serial.print("botAngVel:");
  // Serial.println(vel_Matrix[1][0]);

  //    //PS3 Controller
  // if (Ps3.isConnected()) {
  //   Serial.print("lx:");
  //   Serial.print(Ps3.data.analog.stick.lx);
  //   Serial.print(",");
  //   Serial.print("ly:");
  //   Serial.print(Ps3.data.analog.stick.ly);
  //   Serial.print(",");
  //   Serial.print("rx:");
  //   Serial.print(Ps3.data.analog.stick.rx);
  //   Serial.print(",");
  //   Serial.print("ry:");
  //   Serial.println(Ps3.data.analog.stick.ry);
  // }
}
