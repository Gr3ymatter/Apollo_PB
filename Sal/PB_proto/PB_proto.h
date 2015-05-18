
#ifndef  _PB_proto_H
#define _PB_proto_H

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif

#include "services.h"
#include <lib_aci.h>
#include <aci_setup.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <EEPROM.h>
/**
Put the nRF8001 setup in the RAM of the nRF8001.
*/

#define DEFAULT_REQN 10
#define DEFAULT_RDYN 2
#define DEFAULT_SCK 13
#define DEFAULT_MISO 12
#define DEFAULT_MOSI 11
#define DEFAULT_RST 9
void ble_debugMode(char debugmode);
void ble_begin();
bool bleCanSleep();
void bleWakeUp();
//void ble_set_name(char *name);
bool ble_write(unsigned char data, uint8_t PIPE_NUMBER);
void ble_write_bytes(unsigned char *data, unsigned char len);
void ble_process_events();
//int ble_read();
//unsigned char ble_available();
unsigned char is_ble_connected();
void ble_set_pins(uint8_t SCK, uint8_t MISO, uint8_t MOSI, uint8_t RDYN, uint8_t REQN, uint8_t RST);
//unsigned char ble_busy();

#endif

