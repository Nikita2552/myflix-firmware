#ifndef WIFIACCESS_H
#define WIFIACCESS_H

#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncUDP.h>
#include "globals.h"
#include "estimate.h"

#ifdef DBG
  #include "debug.h"
#endif

void setupWIFI();
void readWIFI();
void readWIFIcallback(AsyncUDPPacket packet);
void parsePCommand(const uint8_t* msg, const size_t len);
void parseCCommand(const uint8_t* msg);
void parsePString(String *str);
void parseParam(String *str, const unsigned int &strnum, const unsigned int &rcnum);

#endif
