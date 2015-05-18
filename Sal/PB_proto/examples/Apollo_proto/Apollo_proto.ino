#include <PB_proto.h>
#include <lib_aci.h>  
#include <SPI.h>
#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
//ble_set_pins(13, 12, 11, 2, 10, 9); // By Default These Pins are compatible with UNO. See Header for sequence.
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
  if(is_ble_connected())
  {
    //written = ble_write(sendbuffer[0], PIPE_USER_DATA_SERVICE_PILL_COUNT_TX);        
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
  Serial.println(F("Arduino Is Sleeping..."));
  delay(100);
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
  /* wake up here */
  sleep_disable(); 
  bleWakeUp();
  delay(100);
  Serial.println(F("Arduino just woke up..."));
} 
  
}
