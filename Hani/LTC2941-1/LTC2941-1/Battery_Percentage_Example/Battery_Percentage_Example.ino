#include <Wire.h>
#include <math.h>


uint8_t columb_Addr = 0x64;
#define LTC_Status 0x00 
#define LTC_Control 0x01
#define LTC_ACC_MSB 0x02
#define LTC_ACC_LSB 0x03
#define LTC_CTH_MSB 0x04
#define LTC_CTH_LSB 0x05
#define LTC_CTL_MSB 0x06
#define LTC_CTL_LSB 0x07
#define e 2.71828183


void setup()
{
Serial.begin(115200);
Wire.begin();
pinMode(4, OUTPUT);

I2C_Write(columb_Addr, LTC_Control, 0x38);
//I2C_Write(columb_Addr, LTC_ACC_MSB, 0x67);
//I2C_Write(columb_Addr, LTC_ACC_LSB, 0x3B);
I2C_Write(columb_Addr, LTC_CTH_MSB, 0xFF);
I2C_Write(columb_Addr, LTC_CTH_LSB, 0xFF);
I2C_Write(columb_Addr, LTC_CTL_MSB, 0x00);
I2C_Write(columb_Addr, LTC_CTL_LSB, 0x00);
}

float batteryVoltage = 0;

void loop()
{  
  batteryVoltage = (float)(analogRead(0)* 5)/1024;
  Serial.print("Battery voltage measured is :");
  Serial.println(batteryVoltage, 4);
  
  uint16_t Charge_DEC;
  Charge_DEC = batteryCharge(batteryVoltage);   
  
  uint8_t batteryLife = batteryPercentage(Charge_DEC);
  Serial.print("Battery Percentage is :");
  Serial.print(batteryLife);
  Serial.println("%");
  
  delay(3000);
  Serial.println(" ");
  
}

void I2C_Write(uint8_t slave_addr, byte reg, byte data)
{ 
  Wire.beginTransmission(slave_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}  

int batteryPercentage(uint16_t x)
{
  uint16_t full  = 65536;
  uint16_t empty = 26427;
  
  float batteryPercent = (x - empty);
  batteryPercent = batteryPercent * 100;
  batteryPercent = batteryPercent/(full - empty);
  batteryPercent = round(batteryPercent);
  return batteryPercent;  
}

uint16_t batteryCharge(uint16_t x)
{
    uint16_t DEC_Value = x;
    
    if(batteryVoltage >= 3.72 && batteryVoltage < 4.04)      // if voltage > 3.72 Volts, y = 199274ln(x) - 212881
    {  
       DEC_Value = 199274.0 * (log(batteryVoltage)/log(e)) - 212881.0;
    }  
    else if (batteryVoltage > 3.55 && batteryVoltage < 3.72)  // if voltage > 3.55 and < 3.72 y = 1.2206e2.836x
    {
      DEC_Value = 1.2206 * exp(2.836*batteryVoltage); //    String test = String(DEC_Value_2, HEX);//    Serial.println(test);
    
    }
    else if (batteryVoltage > 3.1  && batteryVoltage < 3.55)   // if voltage < 3.55 and > 3.1 y = 28184x3 - 267966x2 + 851319x - 876803
    {
      DEC_Value = 28184 * pow (batteryVoltage, 3) - 267966 * pow (batteryVoltage, 2) + 851319 * batteryVoltage - 876803;
    }
    else if (batteryVoltage < 3.1)    // if voltage < 3.1 y = 1242.5x2 - 6510.1x + 34946
    {
      DEC_Value = 1242.5 * pow(batteryVoltage, 2) - 6510.1 * batteryVoltage + 34946;
    }
    
    else
    {
          if (batteryVoltage > 4.091)
            {
              DEC_Value = 65535;
            }
          else
            { 
            //y = 12209x + 15575
            DEC_Value = (12209 * batteryVoltage) + 15575;
            }
    }

 return DEC_Value;  
}






