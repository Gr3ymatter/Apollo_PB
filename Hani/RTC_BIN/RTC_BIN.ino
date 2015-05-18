#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Adafruit_BLE_UART.h"
#include <avr/sleep.h>

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9


uint8_t RTC_addr = 0x68;
#define Seconds 0x00
#define Minutes 0x01  
#define Hours 0x02
#define Day 0x03
#define Date 0x04
#define Month 0x05
#define Year  0x06
#define Control 0x0E
#define StatusRTC 0x0F
#define Alarm1_S 0x07
#define Alarm1_M 0x08
#define Alarm1_H 0x09
#define Alarm1_D 0x0A
#define Alarm2_M 0x0B
#define Alarm2_H 0x0C

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t status;
int addr;


void setup()
{
  Serial.begin(9600);
  BTLEserial.setDeviceName("PillBox"); /* 7 characters max! */
  BTLEserial.begin();
  Wire.begin();
//  attachInterrupt(5, printf, HIGH);  
  attachInterrupt(4, printf, LOW);  


}

void printf()
{
  sleep_disable();
  Serial.println("Pi");
}

void printf2()
{
  sleep_disable();
  Serial.println("Alarm");
}

void loop()
{
  BTLEserial.pollACI();
  status = BTLEserial.getState();
  if (status != laststatus) 
  {
    if (status == ACI_EVT_DEVICE_STARTED)
    {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) 
    {
        
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) 
    {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    laststatus = status;
  }
  
if (status == ACI_EVT_CONNECTED && BTLEserial.available() && (((I2C_Read(RTC_addr, 0x0F)) & 0x80) == 128))
{   
  char c[12];
  uint8_t i = 0;
  while (BTLEserial.available()) 
  { 
    c[i] =  BTLEserial.read();
    i++;
  }
  
  byte M; byte D; byte Y; byte N; byte S; byte H;
  RTC_Setup(&M, &D, &Y, &H, &N, &S, c);
  I2C_Write(RTC_addr, Month, M);
  I2C_Write(RTC_addr, Date, D);
  I2C_Write(RTC_addr, Year, Y);
  I2C_Write(RTC_addr, Hours, H);
  I2C_Write(RTC_addr, Minutes, N);
  I2C_Write(RTC_addr, Seconds, S);
  I2C_Write(RTC_addr, StatusRTC, 0x08);        
}

if (status == ACI_EVT_CONNECTED && BTLEserial.available() && (((I2C_Read(RTC_addr, 0x0F)) & 0x80) != 128))

{
  char c[6];
  uint8_t i = 0;
  while (BTLEserial.available()) 
  { 
    c[i] =  BTLEserial.read();
    i++;
  }
  
  byte H; byte M; byte S; 
  Alarm_Setup(&H, &M, &S, c);
  I2C_Write(RTC_addr, Alarm1_H, H);
  I2C_Write(RTC_addr, Alarm1_M, M);
  I2C_Write(RTC_addr, Alarm1_S, S);
  I2C_Write(RTC_addr, Alarm1_D, 0x80);
  I2C_Write(RTC_addr, Control, 0b00000101);
  
  Serial.println("Alarm Set");
  delay(200);
  sleepNow();

} 


//if (((I2C_Read(RTC_addr, 0x0F)) & 0x80) == 128 && status == ACI_EVT_CONNECTED)
//{
//  BTE_send("Program RTC");
//}

if (status == ACI_EVT_CONNECTED && digitalRead(14) == HIGH)
{
   delay(100);
   String cTime = currentTime();
   Serial.println(cTime);
   BTE_send(cTime);
   delay(200);
  // sleepNow();
   
}

else if (status != ACI_EVT_CONNECTED && digitalRead(14) == HIGH)

{
  delay(100);
  String cTime = currentTime();
  EEPROM_Write(cTime, &addr);  
}
  
//  else if(((I2C_Read(RTC_addr, 0x0F)) & 0x80) != 128)
//  {
//  Serial.print(I2C_Read(RTC_addr, Month));
//  Serial.print("/");
//  Serial.print(bcd2bin(I2C_Read(RTC_addr, Date)));
//  Serial.print("/");
//  Serial.println(bcd2bin(I2C_Read(RTC_addr, Year)));
//  Serial.print(bcd2bin(I2C_Read(RTC_addr, Hours)));
//  Serial.print(": ");
//  Serial.print(bcd2bin(I2C_Read(RTC_addr, Minutes)));
//  Serial.print(": ");
//  Serial.print(bcd2bin(I2C_Read(RTC_addr, Seconds)));
//  Serial.println(" ");
//  Serial.print((I2C_Read(RTC_addr, StatusRTC)));
//  Serial.println(" ");
//  }
//delay(1000);
}

void EEPROM_Write(String t, int *addr)
{ 
  for (int i = 0; i < t.length(); i++)
   { 
     EEPROM.write(*addr, t[i]);
     Serial.print("address is : ");
     Serial.println(*addr);
     Serial.print("value is : ");
     Serial.println(t[i]);
     *addr = *addr + 1;
   }  
}

void RTC_Setup (byte *M, byte *D, byte *Y, byte *H, byte *N, byte *S, char c[12])
{ 
  *M = c[0] - 48;
  *M = *M << 4;
  *M = *M + (c[1] - 48);
  
  *D = c[2] - 48;
  *D = *D << 4;
  *D = *D + (c[3] - 48);
  
  *Y = c[4] - 48;
  *Y = *Y << 4;
  *Y = *Y + (c[5] - 48);
  
  *H = c[6] - 48;
  *H = *H << 4;
  *H = *H + (c[7] - 48);
  
  *N = c[8] - 48;
  *N = *N << 4;
  *N = *N + (c[9] - 48);
  
  *S = c[10] - 48;
  *S = *S << 4;
  *S = *S + (c[11] - 48);
}

void Alarm_Setup(byte *H, byte *M, byte *S, char c[6])
{
  *H = c[0] - 48;
  *H = *H << 4;
  *H = *H + (c[1] - 48);
  
  *M = c[2] - 48;
  *M = *M << 4;
  *M = *M + (c[3] - 48);
  
  *S = c[4] - 48;
  *S = *S << 4;
  *S = *S + (c[5] - 48);
}

uint8_t bcd2bin (uint8_t val)
{
    return val - 6 * (val >> 4);
}  
  
void I2C_Write(uint8_t RTC_addr, byte reg, byte data)
{ 
  Wire.beginTransmission(RTC_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}  

byte I2C_Read(uint8_t RTC_addr, byte reg)
{ 
  uint8_t var = 1;
  Wire.beginTransmission(RTC_addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(RTC_addr, var);  
  byte temp = Wire.read(); 
  return temp;
}

void BTE_send(String s)
{
  uint8_t sendbuffer[20];
  s.getBytes(sendbuffer, 20);
  char sendbuffersize = min(20, s.length()); 
  BTLEserial.write(sendbuffer, sendbuffersize);
}

String currentTime()
{
  
   String oTime =  (String) bcd2bin(I2C_Read(RTC_addr, Month));
   oTime = oTime + "/" + (String) bcd2bin(I2C_Read(RTC_addr, Date));
   oTime = oTime + "/" + (String) bcd2bin(I2C_Read(RTC_addr, Year));
   oTime = oTime + " " + (String) bcd2bin(I2C_Read(RTC_addr, Hours));
   oTime = oTime + ":" + (String) bcd2bin(I2C_Read(RTC_addr, Minutes));
   oTime = oTime + ":" + (String) bcd2bin(I2C_Read(RTC_addr, Seconds));
   return oTime;
  
}

void sleepNow()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();
  sleep_cpu();            // here the device is actually put to sleep!!
  Serial.println("woke up");
  Serial.println(digitalRead(2));
}
