#include <PB_NRF8001.h>
#include <lib_aci.h>  
#include <SPI.h>
#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
ble_set_pins(13, 12, 11, 2, 10, 9); // By Default These Pins are compatible with UNO. See Header for sequence.
ble_debugMode(1); // Print Codes
ble_bondMode(1); //Initiate Bonding Mode 
ble_begin();     // Begin BLE
}
void loop() {
  ble_do_events();
}
