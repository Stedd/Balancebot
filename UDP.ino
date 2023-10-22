const char* ssid = SECRET_SSID;
const char* password = SECRET_PASSWORD;

#include "WiFi.h"
#include "AsyncUDP.h"

const IPAddress multicastIP = IPAddress(239, 1, 2, 3);
int port = 1234;

byte watchdog = 0;
AsyncUDP udp;

void UdpInit() {
  ConnectToWiFi();
}

void UdpLoop() {
  PackUdpData();
  udp.writeTo(data, sizeof(data), multicastIP, port);
}

void ConnectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
}

void PackUdpData() {
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
