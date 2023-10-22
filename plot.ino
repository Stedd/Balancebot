void plot() {
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

  int i = 0;
  data[i] = watchdog++;
  data[i += 1] = balancingOn << 1;
  i = PackInt(i += 1, M1_Speed_CMD);
  i = PackInt(i, M2_Speed_CMD);
  i = PackFloat(i, acc_pitch);
  i = PackFloat(i, pitch);
  i = PackFloat(i, pitch_rate);
  i = PackFloat(i, rem_speed_ref);
  i = PackFloat(i, rem_turn_speed_ref);
  i = PackFloat(i, SC_cont_out);
  i = PackFloat(i, TC_cont_out);
  i = PackFloat(i, OL_cont_out);
  i = PackFloat(i, ref_IL);
  i = PackFloat(i, act_IL);
  i = PackFloat(i, error_IL);
  i = PackFloat(i, IL_cont_out);
  i = PackFloat(i, iError_IL);
  i = PackFloat(i, IL_anti_windup);
  i = PackFloat(i, speedCmd1);
  i = PackFloat(i, speedCmd2);
  i = PackFloat(i, vel_Matrix[0][0]);
  i = PackFloat(i, vel_Matrix[1][0]);
  i = PackFloat(i, motor_ang_vel[0][0]);
  i = PackFloat(i, motor_ang_vel[1][0]);
  i = PackLong(i, m1Raw);
  i = PackLong(i, m2Raw);
}

int PackInt(int _i, int value) {
  data[_i] = (value & 0x00FF);
  data[_i + 1] = (value & 0xFF00) >> 8;
  return _i + 2;
}

int PackLong(int _i, long value) {
  data[_i] = (value & 0x000000FF);
  data[_i + 1] = (value & 0x0000FF00) >> 8;
  data[_i + 2] = (value & 0x00FF0000) >> 16;
  data[_i + 3] = (value & 0xFF000000) >> 24;
  return _i + 4;
}

union FloatToBytes {
  float value;
  byte bytes[4];
};

int PackFloat(int _i, float value) {
  FloatToBytes converter;
  converter.value = value;
  for (int j = 0; j < 4; j++) {
    data[_i + j] = converter.bytes[j];
  }
  return _i + 4;
}



// int PackFloat(int _i, float value) {
//   data[_i] = (value & 0x000000FF) ;
//   data[_i + 2] = (value & 0x0000FF00)>>8;
//   data[_i + 3] = (value & 0x00FF0000)>>16;
//   data[_i + 4] = (value & 0xFF000000)>>24;
//   return _i + 4;
// }
