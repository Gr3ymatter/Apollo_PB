#include <Wire.h>

uint8_t columb_Addr = 0x64;
#define LTC_Status 0x00 
#define LTC_Control 0x01
#define LTC_ACC_MSB 0x02
#define LTC_ACC_LSB 0x03
#define LTC_CTH_MSB 0x04
#define LTC_CTH_LSB 0x05
#define LTC_CTL_MSB 0x06
#define LTC_CTL_LSB 0x07


void setup()
{
Serial.begin(9600);
Wire.begin();

I2C_Write(columb_Addr, LTC_Control, 0x38);
I2C_Write(columb_Addr, LTC_ACC_MSB, 0x67);
I2C_Write(columb_Addr, LTC_ACC_LSB, 0x3B);
I2C_Write(columb_Addr, LTC_CTH_MSB, 0xFF);
I2C_Write(columb_Addr, LTC_CTH_LSB, 0xFF);
I2C_Write(columb_Addr, LTC_CTL_MSB, 0x00);
I2C_Write(columb_Addr, LTC_CTL_LSB, 0x00);
}

double batteryVoltage = 0;

void loop()
{
  batteryVoltage = analogRead(0);
  Serial.print(batteryVoltage);
  Serial.print("\t");
  
  
  Wire.requestFrom(100, 8);    // request 6 bytes from slave device #2
  int i = 0;
  while (Wire.available())   // slave may send less than requested
  {
    byte c = Wire.read(); // receive a byte as character
    Serial.print(c, HEX);         // print the character
    Serial.print("\t");
    i++;
  }
   Serial.println();
  delay(2000);
}

void I2C_Write(uint8_t slave_addr, byte reg, byte data)
{ 
  Wire.beginTransmission(slave_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}  

//byte I2C_Read(uint8_t slave_addr, byte reg)
//{ 
//  uint8_t var = 1;
//  Wire.beginTransmission(slave_addr);
//  Wire.write(reg);
//  Wire.endTransmission();
//  Wire.requestFrom(slave_addr, 8);  
//  byte temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  temp = Wire.read(); 
//  Serial.println(temp);
//  return temp;
//  delay(1000);
//}

