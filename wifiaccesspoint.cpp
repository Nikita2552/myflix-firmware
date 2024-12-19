#include "wifiaccesspoint.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#endif

AsyncUDP udp;
const uint16_t port = 49152;

const char *ssid = "yourAP";
const char *password = "yourPassword";

vector<int> rc{0, 0, 0, 0}; //Данные с пульта для корректировки дрона
bool newDataFlag = false;

//vector<float> targetAngle{0, 0, 1};
//float thrust = 0;

void setupWIFI()
{
#ifdef DBG
  printDEBUG("Configuring access point...");
#endif

  pinMode(LED_BUILTIN, OUTPUT);

  if (!WiFi.softAP(ssid, password))
  {
    log_e("Soft AP creation failed.");
    while (1);
  }

  IPAddress myIP = WiFi.softAPIP();
  
#ifdef DBG
  printDEBUG("AP IP address: ");
  printlnDEBUG(myIP);
#endif

  if (udp.listen(port))
  {
    udp.onPacket(readWIFIcallback);
  }

#ifdef DBG
  printlnDEBUG("Server started");
#endif
}

void readWIFI()
{

}

void readWIFIcallback(AsyncUDPPacket packet)
{
  // Записываем адрес начала данных в памяти
  const uint8_t* msg = packet.data();
  // Записываем размер данных
  const size_t len = packet.length();

  Serial.println("Client connected.");

  // Если адрес данных не равен нулю и размер данных больше нуля...
  if (msg != NULL && len > 0)
  {                      
    Serial.println("New Client.");
        
    if (msg[0] == 'P')
    {
      parsePCommand(msg, len);
    }
    else if (msg[0] == 'C')
    {
      parseCCommand(msg);
    }

    Serial.println("Client Disconnected.");
  }  
}

void parsePCommand(const uint8_t* msg, const size_t len)
{  
  char parameter;
  String currentLine;
  
  char i = 0;
  while ((parameter != '.') && (len < 24))
  {
    parameter = *(msg + i);
    currentLine += parameter;
    i++; 
  }

  if (parameter == '.')
  {
    printlnDEBUG(currentLine);
    parsePString(&currentLine);
    printlnDEBUG("parsing position is duing");
  } 
}

void parseCCommand(const uint8_t* msg)
{
  char parameter = *msg;  

  if (parameter == '.')
  {    
    callibrate();
    printlnDEBUG("callibration is duing");
  }
}

void parsePString(String *str)
{
  if ((str->charAt(1) == ':') &&
      (str->charAt(2) == 'Y') &&
      (str->charAt(7) == 'R') &&
      (str->charAt(12) == 'P') &&
      (str->charAt(17) == 'T'))
  {    
    parseParam(str, 3, 0);
    parseParam(str, 8, 1);
    parseParam(str, 13, 2);
    parseParam(str, 18, 3);

    printlnDEBUG("rc:"+
       String(rc[0])+", "+
       String(rc[1])+", "+
       String(rc[2])+", "+
       String(rc[3])+ ";");

    newDataFlag = true; 
  } 
}

void parseParam(String *str, const unsigned int &strnum, const unsigned int &rcnum)
{
  if ((rcnum < RCNUM) && (strnum < str->length()))
  {
    String param;
    int i;

    i = 1;    

    while (i <= 3)
    {
      param += str->charAt(strnum + i);
      i++;
    }

    i = 1;

    if (str->charAt(strnum) == '-')
      i = -1;

    rc[rcnum] = param.toInt() * i;
  }  
}