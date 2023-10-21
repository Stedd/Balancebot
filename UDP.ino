const char* ssid = "CaveBot";
const char* password = "&nHM%D2!$]Qg[VUv";

#include "WiFi.h"
#include "AsyncUDP.h"

const IPAddress multicastIP = IPAddress(239, 1, 2, 3);
int port = 1234;

byte watchdog = 0;
AsyncUDP udp;

void UdpInit() {
  //Serial.begin(115200);
  ConnectToWiFi();
  //udp.connect(multicastIP, port);
}

void UdpLoop() {
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
