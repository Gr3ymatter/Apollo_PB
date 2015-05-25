#include <PB_proto.h>
#include <lib_aci.h>  
#include <SPI.h>
#include <EEPROM.h>

Prescription prescription1;
Prescription prescription2;
Prescription prescription3;
Prescription prescription4;

void setup() {
  // put your setup code here, to run once:
//ble_set_pins(13, 12, 11, 2, 10, 9); // By Default These Pins are compatible with UNO. See Header for sequence.
ble_debugMode(0); // Print Codes
ble_begin();     // Begin BLE
prescription1.Pill_Compartment = 1;
prescription2.Pill_Compartment = 2;
prescription3.Pill_Compartment = 3;
prescription4.Pill_Compartment = 4;

}
void loop() {
  
  ble_process_events(&prescription1, &prescription2, &prescription3, &prescription4);  
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
  
  
 if(ble_isDataAvailable(&prescription1)){ 
 Serial.println(F("**Prescription Information Recieved**"));
 Serial.print(F("Prescription Name: "));
 Serial.println(prescription1.Pill_Name.pname);
 Serial.print(F("Prescription Compartment: "));
 Serial.println(prescription1.Pill_Compartment);
 Serial.print(F("Prescription Count: "));
 Serial.println(prescription1.Pill_Count.dataValue);
 Serial.print(F("Prescription Dosage: "));
 Serial.println(prescription1.Pill_Dosage.dataValue);
 Serial.print(F("Alarm Number "));
 Serial.print(prescription1.Pill_Time.alarmNumber); 
 Serial.print(F(" prescription time: "));
 Serial.println(prescription1.Pill_Time.data.dataValue);
 }

 if(ble_isDataAvailable(&prescription2)){ 
 Serial.println(F("**Prescription Information Recieved**"));
 Serial.print(F("Prescription Name: "));
 Serial.println(prescription2.Pill_Name.pname);
 Serial.print(F("Prescription Compartment: "));
 Serial.println(prescription2.Pill_Compartment);
 Serial.print(F("Prescription Count: "));
 Serial.println(prescription2.Pill_Count.dataValue);
 Serial.print(F("Prescription Dosage: "));
 Serial.println(prescription2.Pill_Dosage.dataValue);
 Serial.print(F("Alarm Number "));
 Serial.print(prescription2.Pill_Time.alarmNumber); 
 Serial.print(F(" prescription time: "));
 Serial.println(prescription2.Pill_Time.data.dataValue);
 }
 
  if(ble_isDataAvailable(&prescription3)){ 
 Serial.println(F("**Prescription Information Recieved**"));
 Serial.print(F("Prescription Name: "));
 Serial.println(prescription3.Pill_Name.pname);
 Serial.print(F("Prescription Compartment: "));
 Serial.println(prescription3.Pill_Compartment);
 Serial.print(F("Prescription Count: "));
 Serial.println(prescription3.Pill_Count.dataValue);
 Serial.print(F("Prescription Dosage: "));
 Serial.println(prescription3.Pill_Dosage.dataValue);
 Serial.print(F("Alarm Number "));
 Serial.print(prescription3.Pill_Time.alarmNumber); 
 Serial.print(F(" prescription time: "));
 Serial.println(prescription3.Pill_Time.data.dataValue);
 }
 
  if(ble_isDataAvailable(&prescription4)){ 
 Serial.println(F("**Prescription Information Recieved**"));
 Serial.print(F("Prescription Name: "));
 Serial.println(prescription4.Pill_Name.pname);
 Serial.print(F("Prescription Compartment: "));
 Serial.println(prescription4.Pill_Compartment);
 Serial.print(F("Prescription Count: "));
 Serial.println(prescription4.Pill_Count.dataValue);
 Serial.print(F("Prescription Dosage: "));
 Serial.println(prescription4.Pill_Dosage.dataValue);
 Serial.print(F("Alarm Number "));
 Serial.print(prescription4.Pill_Time.alarmNumber); 
 Serial.print(F(" prescription time: "));
 Serial.println(prescription4.Pill_Time.data.dataValue);
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
