#include <PB_proto.h>
#include <lib_aci.h>  
#include <SPI.h>
#include <EEPROM.h>

/*
ble_set_pins(SCLK, MISO, MOSI, READY, SS, RESET);
ble_set_pins(13, 12, 11, 2, 10, 9); // By Default These Pins are compatible with UNO. See Header for sequence.

Serial in the library is set to 115200

UNO SPI:        Mega SPI      AdaFruit NRF8001
SCLK - pin 13     pin 52   |   RESET - pin 4
MISO - pin 12     pin 50   |   READY - pin 2 (interrupt)
MOSI - pin 11     pin 51   
SS - pin 10       pin 53

pin 6 is the BLE enable and bonding terminal. Connect it to GND for operation.
Disconnect from GND to clear the bonding memory
Connect it back to GND to enable advertising and Re-Open the Serial console.

Service: User Data Service - Characteristic: Pill Count - Pipe: TX 
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_TX          4
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_TX_MAX_SIZE 4

 Service: User Data Service - Characteristic: Pill Count - Pipe: RX 
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_RX          5
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_RX_MAX_SIZE 4

Service: User Data Service - Characteristic: Pill Name - Pipe: RX
#define PIPE_USER_DATA_SERVICE_PILL_NAME_RX          7
#define PIPE_USER_DATA_SERVICE_PILL_NAME_RX_MAX_SIZE 15

Service: User Data Service - Characteristic: Pill Dosage - Pipe: RX
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_RX          9
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_RX_MAX_SIZE 4

Service: User Data Service - Characteristic: Pill Time - Pipe: TX 
#define PIPE_USER_DATA_SERVICE_PILL_TIME_TX          11
#define PIPE_USER_DATA_SERVICE_PILL_TIME_TX_MAX_SIZE 10

Service: User Data Service - Characteristic: Pill Time - Pipe: RX 
#define PIPE_USER_DATA_SERVICE_PILL_TIME_RX          12
#define PIPE_USER_DATA_SERVICE_PILL_TIME_RX_MAX_SIZE 10

*/

void setup() {
  // put your setup code here, to run once:
//ble_set_pins(13, 12, 11, 2, 10, 9);  
ble_set_pins(52, 50, 51, 2, 53, 4); // By Default These Pins are compatible with UNO. See Header for sequence.
ble_debugMode(1); // Print Codes
ble_begin();     // Begin BLE
}
void loop() {
  
  ble_process_events();  

  if(Serial.available())
  {
   bool written = false;
   Serial.setTimeout(100);
   String s = Serial.readString();   
   
   uint8_t sendbuffer[20];
   s.getBytes(sendbuffer, 20);
   uint8_t sendbuffersize = min(20, s.length());
   
//   Serial.print("Buffer Size: ");
//   Serial.println(sendbuffersize);
//   
//   Serial.print("Data is: ");
//   Serial.println(*sendbuffer);
   
   
  if(is_ble_connected())
  {
    written = ble_write(*sendbuffer, PIPE_USER_DATA_SERVICE_PILL_COUNT_TX, sendbuffersize);        
  } 
  if(written)
  {
    
    Serial.println(F("String Sent"));
    
  }
  else
  {
    Serial.println(F("String Not Sent")); 
  }
  }
  
  
 if(bleCanSleep())
{
//  Serial.println(F("Arduino Is Sleeping..."));
  delay(900);
//  sleep_enable();
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  sleep_cpu();
  /* wake up here */
//  sleep_disable(); 
//  bleWakeUp();
  delay(100);
//  Serial.println(F("Arduino just woke up..."));
} 
  
}
