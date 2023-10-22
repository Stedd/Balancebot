void SerialPlot() {
  // Time
  // Serial.print("dT:");
  // Serial.println(dT);
  // Serial.print(" ");
  // Serial.print("dT_s:");
  // Serial.println(dT_s);
  // Serial.print(" ");

  // IMU

  // Serial.print("RollRate:");
  // Serial.println(pitch_rate);

  // Serial.print("Accelerometer_Pitch:");
  // Serial.println(acc_pitch);

  // Serial.print("Pitch:");
  // Serial.println(pitch);
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
  // Serial.println(m1Raw);

  // Serial.print("m2Raw:");
  // Serial.println(m2Raw);

  // // Motors
  // Serial.print("SpeedControllerOut:");
  // Serial.println(SC_cont_out);

  // Serial.print("BalanceOLControllerOut:");
  // Serial.println(OL_cont_out);

  // Serial.print("BalanceILControllerOut:");
  // Serial.println(IL_cont_out);

  // Serial.print("TurnControllerOut:");
  // Serial.println(TC_cont_out);

  // Serial.print("M1_CMD:");
  // Serial.println(M1_Speed_CMD);

  // Serial.print("M2_CMD:");
  // Serial.println(M2_Speed_CMD);

  // Serial.print("M1_Ang_Vel:");
  // Serial.print(motor_ang_vel[0][0]);
  // Serial.print(" ");
  // Serial.print("M2_Ang_Vel:");
  // Serial.print(motor_ang_vel[0][1]);
  // Serial.print(" ");
  // Serial.print("botLinVel:");
  // Serial.println(vel_Matrix[0][0]);
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



// int PackFloat(int _i, float value) {
//   data[_i] = (value & 0x000000FF) ;
//   data[_i + 2] = (value & 0x0000FF00)>>8;
//   data[_i + 3] = (value & 0x00FF0000)>>16;
//   data[_i + 4] = (value & 0xFF000000)>>24;
//   return _i + 4;
// }
