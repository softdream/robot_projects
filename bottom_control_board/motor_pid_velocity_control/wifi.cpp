#include "wifi.h"

const char *ssid = "1234567";
const char *password = "xd123456";
WiFiUDP udp, udp2;
IPAddress remote_IP(192, 168, 43, 190); // udp 服务端ip
unsigned int remote_port = 2333;
unsigned int remote_port2 = 2335;

void wifiInit()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(2500);
    //Serial.print(".");
    WiFi.begin(ssid, password);
  }
  //Serial.print("Connected, IP : ");
  //Serial.println(WiFi.localIP());
}
