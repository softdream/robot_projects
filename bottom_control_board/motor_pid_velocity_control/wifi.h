#ifndef __WIFI_H
#define __WIFI_H

#include <WiFi.h>

extern WiFiUDP udp, udp2;
extern IPAddress remote_IP;
extern unsigned int remote_port;
extern unsigned int remote_port2;

void wifiInit();

#endif
