
#include <Arduino.h>
#include "PB_proto.h"
#include "services.h"


#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;



// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

static bool timing_change_done          = false;
static unsigned char is_connected = 0;
static unsigned char debugMode = 0;

/*
We will store the bonding info for the nRF8001 in the MCU to recover from a power loss situation
*/
static bool bonded_first_time = true;
static unsigned char gotCMDRSP = 0;
static unsigned char gotData = 0;

uint8_t reqn_pin = DEFAULT_REQN;
uint8_t rdyn_pin = DEFAULT_RDYN;
uint8_t sck_pin = DEFAULT_SCK;
uint8_t miso_pin = DEFAULT_MISO;
uint8_t mosi_pin = DEFAULT_MOSI;
uint8_t rst_pin = DEFAULT_RST;

void ble_debugMode(char debugmode){
	debugMode = debugmode;
}

 void ble_set_pins(uint8_t SCK, uint8_t MISO, uint8_t MOSI, uint8_t RDYN, uint8_t REQN, uint8_t RST)
{
    sck_pin = SCK;
	miso_pin = MISO;
	mosi_pin = MOSI;
	rst_pin = RST;
	reqn_pin = REQN;
    rdyn_pin = RDYN;
}

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

/*
Read the Dymamic data from the EEPROM and send then as ACI Write Dynamic Data to the nRF8001
This will restore the nRF8001 to the situation when the Dynamic Data was Read out
*/
aci_status_code_t bond_data_restore(aci_state_t *aci_stat, uint8_t eeprom_status, bool *bonded_first_time_state)
{
  aci_evt_t *aci_evt;
  uint8_t eeprom_offset_read = 1;
  uint8_t write_dyn_num_msgs = 0;
  uint8_t len =0;


  // Get the number of messages to write for the eeprom_status
  write_dyn_num_msgs = eeprom_status & 0x7F;

  //Read from the EEPROM
  while(1)
  {
    len = EEPROM.read(eeprom_offset_read);
    eeprom_offset_read++;
    aci_cmd.buffer[0] = len;

    for (uint8_t i=1; i<=len; i++)
    {
        aci_cmd.buffer[i] = EEPROM.read(eeprom_offset_read);
        eeprom_offset_read++;
    }
    //Send the ACI Write Dynamic Data
    if (!hal_aci_tl_send(&aci_cmd))
    {
      Serial.println(F("bond_data_restore: Cmd Q Full"));
      return ACI_STATUS_ERROR_INTERNAL;
    }

    //Spin in the while loop waiting for an event
    while (1)
    {
      if (lib_aci_event_get(aci_stat, &aci_data))
      {
        aci_evt = &aci_data.evt;

        if (ACI_EVT_CMD_RSP != aci_evt->evt_opcode)
        {
          //Got something other than a command response evt -> Error
          Serial.print(F("bond_data_restore: Expected cmd rsp evt. Got: 0x"));
          Serial.println(aci_evt->evt_opcode, HEX);
          return ACI_STATUS_ERROR_INTERNAL;
        }
        else
        {
          write_dyn_num_msgs--;

          //ACI Evt Command Response
          if (ACI_STATUS_TRANSACTION_COMPLETE == aci_evt->params.cmd_rsp.cmd_status)
          {
            //Set the state variables correctly
            *bonded_first_time_state = false;
            aci_stat->bonded = ACI_BOND_STATUS_SUCCESS;

            return ACI_STATUS_TRANSACTION_COMPLETE;
          }
          if (0 >= write_dyn_num_msgs)
          {
            //should have returned earlier
            return ACI_STATUS_ERROR_INTERNAL;
          }
          if (ACI_STATUS_TRANSACTION_CONTINUE == aci_evt->params.cmd_rsp.cmd_status)
          {
            //break and write the next ACI Write Dynamic Data
            break;
          }
        }
      }
    }

  }
}

/*
This function is specific to the atmega328
@params ACI Command Response Evt received from the Read Dynmaic Data
*/
void bond_data_store(aci_evt_t *evt)
{
  static int eeprom_write_offset = 1;

  //Write it to non-volatile storage
  EEPROM.write( eeprom_write_offset, evt->len -2 );
  eeprom_write_offset++;

  EEPROM.write( eeprom_write_offset, ACI_CMD_WRITE_DYNAMIC_DATA);
  eeprom_write_offset++;

  for (uint8_t i=0; i< (evt->len-3); i++)
  {
    EEPROM.write( eeprom_write_offset, evt->params.cmd_rsp.params.padding[i]);
    eeprom_write_offset++;
  }
}


bool bond_data_read_store(aci_state_t *aci_stat)
{
  /*
  The size of the dynamic data for a specific Bluetooth Low Energy configuration
  is present in the ublue_setup.gen.out.txt generated by the nRFgo studio as "dynamic data size".
  */
  bool status = false;
  aci_evt_t * aci_evt = NULL;
  uint8_t read_dyn_num_msgs = 0;

  //Start reading the dynamic data
  lib_aci_read_dynamic_data();
  read_dyn_num_msgs++;

  while (1)
  {
    if (true == lib_aci_event_get(aci_stat, &aci_data))
    {
      aci_evt = &aci_data.evt;

      if (ACI_EVT_CMD_RSP != aci_evt->evt_opcode )
      {
        //Got something other than a command response evt -> Error
        status = false;
        break;
      }

      if (ACI_STATUS_TRANSACTION_COMPLETE == aci_evt->params.cmd_rsp.cmd_status)
      {
        //Store the contents of the command response event in the EEPROM
        //(len, cmd, seq-no, data) : cmd ->Write Dynamic Data so it can be used directly
        bond_data_store(aci_evt);

        //Set the flag in the EEPROM that the contents of the EEPROM is valid
        EEPROM.write(0, 0x80|read_dyn_num_msgs );
        //Finished with reading the dynamic data
        status = true;

        break;
      }

      if (!(ACI_STATUS_TRANSACTION_CONTINUE == aci_evt->params.cmd_rsp.cmd_status))
      {
        //We failed the read dymanic data
        //Set the flag in the EEPROM that the contents of the EEPROM is invalid
        EEPROM.write(0, 0x00);

        status = false;
        break;
      }
      else
      {
        //Store the contents of the command response event in the EEPROM
        // (len, cmd, seq-no, data) : cmd ->Write Dynamic Data so it can be used directly when re-storing the dynamic data
        bond_data_store(aci_evt);

        //Read the next dynamic data message
        lib_aci_read_dynamic_data();
        read_dyn_num_msgs++;
      }

    }
  }
  return status;
}


unsigned char is_ble_connected(){
	return is_connected;
}


//Process all ACI events here
//Process all ACI events here
void ble_process_events()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

	Serial.print(F("Event OpCode = "));
	Serial.println(aci_evt->evt_opcode, HEX);
	
    switch(aci_evt->evt_opcode)
    {
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            aci_state.device_state = ACI_DEVICE_SETUP;
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
              //Manage the bond in EEPROM of the AVR
              {
                uint8_t eeprom_status = 0;
                eeprom_status = EEPROM.read(0);
                if (eeprom_status != 0x00)
                {
                  Serial.println(F("Previous Bond present. Restoring"));
                  Serial.println(F("Using existing bond stored in EEPROM."));
                  Serial.println(F("   To delete the bond stored in EEPROM, connect Pin 6 to 3.3v and Reset."));
                  Serial.println(F("   Make sure that the bond on the phone/PC is deleted as well."));
                  //We must have lost power and restarted and must restore the bonding infromation using the ACI Write Dynamic Data
                  if (ACI_STATUS_TRANSACTION_COMPLETE == bond_data_restore(&aci_state, eeprom_status, &bonded_first_time))
                  {
                    Serial.println(F("Bond restored successfully"));
                  }
                  else
                  {
                    Serial.println(F("Bond restore failed. Delete the bond and try again."));
                  }
                }
              }

              // Start bonding as all proximity devices need to be bonded to be usable
              if (ACI_BOND_STATUS_SUCCESS != aci_state.bonded)
              {
                lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
                Serial.println(F("No Bond present in EEPROM."));
                Serial.println(F("Advertising started : Waiting to be connected and bonded"));
              }
              else
              {
                //connect to an already bonded device
                //Use lib_aci_direct_connect for faster re-connections with PC, not recommended to use with iOS/OS X
				delay(100);
                lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
                Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
              }
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
		Serial.println(F("ACI CMD RSP"));
        if ((ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status )
            && (ACI_CMD_READ_DYNAMIC_DATA  != aci_evt->params.cmd_rsp.cmd_opcode)
            && (ACI_CMD_WRITE_DYNAMIC_DATA != aci_evt->params.cmd_rsp.cmd_opcode))
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command

          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
             (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
		gotCMDRSP = 1;
        break;

      case ACI_EVT_CONNECTED:
        /*
        reset the credit available when the link gets connected
        */
        aci_state.data_credit_available = aci_state.data_credit_total;
        Serial.println(F("Evt Connected"));
        /*
         Get the Device Version of the nRF8001 and place it in the
         Hardware Revision String Characteristic of the Device Info. GATT Service
         */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (
            (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); //Uses the GAP preferred timing as put in the nRFGo studio xml file-> See also in services.h
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.print(F("Timing change received conn Interval: 0x"));
        Serial.println(aci_evt->params.timing.conn_rf_interval, HEX);
        //Disconnect as soon as we are bonded and required pipes are available
        //This is used to store the bonding info on disconnect and then re-connect to verify the bond
        if((ACI_BOND_STATUS_SUCCESS == aci_state.bonded) &&
           (true == bonded_first_time) &&
           (GAP_PPCP_MAX_CONN_INT >= aci_state.connection_interval) &&
           (GAP_PPCP_MIN_CONN_INT <= aci_state.connection_interval) //Timing change already done: Provide time for the the peer to finish
           )
           {
             lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
           }
        break;

      case ACI_EVT_DATA_CREDIT:
        /**
        Bluetooth Radio ack received from the peer radio for the data packet sent.
        Multiple data packets can be acked in a single aci data credit event.
        */
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_BOND_STATUS:
        Serial.println(F("Evt Bond Status"));
        aci_state.bonded = aci_evt->params.bond_status.status_code;
        break;

      case ACI_EVT_DISCONNECTED:
        /**
        Advertise again if the advertising timed out.
        */
        if(ACI_STATUS_ERROR_ADVT_TIMEOUT == aci_evt->params.disconnected.aci_status)
        {
          Serial.println(F("Evt Disconnected -> Advertising timed out"));
          {
            Serial.println(F("nRF8001 going to sleep"));
            lib_aci_sleep();
            aci_state.device_state = ACI_DEVICE_SLEEP;
            //Put the MCU to sleep here
            // Wakeup the MCU and the nRF8001 when the joystick button is pressed
            // Use lib_aci_device_wakeup() to wakeup the nRF8001
          }
        }
        else
        {
          if (ACI_BOND_STATUS_SUCCESS != aci_state.bonded)
          {
            // Previous bonding failed. Try to bond again.
            lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
            Serial.println(F("Advertising started : Waiting to be connected and bonded"));
          }
          else
          {
            if (bonded_first_time)
            {
              bonded_first_time = false;
              //Store away the dynamic data of the nRF8001 in the Flash or EEPROM of the MCU
              // so we can restore the bond information of the nRF8001 in the event of power loss
              if (bond_data_read_store(&aci_state))
              {
                Serial.println(F("Dynamic Data read and stored successfully"));
              }
            }

            //connect to an already bonded device
            //Use lib_aci_direct_connect for faster re-connections (advertising interval of 3.75 ms is used for directed advertising)
            lib_aci_connect(180/* in seconds */, 0x0020 /* advertising interval 20ms*/);
            Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
          }
        }
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe #: 0x"));
        Serial.print(aci_evt->params.data_received.rx_data.pipe_number, HEX);
        {
          int i;
          Serial.print(F(" Data(Hex) : "));
          for(i=0; i<aci_evt->len - 2; i++)
          {
            Serial.print(aci_evt->params.data_received.rx_data.aci_data[i], HEX);
            Serial.print(F(" "));
          }
        }
        Serial.println(F(""));
		gotData = 1;
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
        Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();

        //Manage the bond in EEPROM of the AVR
        {
          uint8_t eeprom_status = 0;
          eeprom_status = EEPROM.read(0);
          if (eeprom_status != 0x00)
          {
            Serial.println(F("Previous Bond present. Restoring"));
            Serial.println(F("Using existing bond stored in EEPROM."));
            Serial.println(F("   To delete the bond stored in EEPROM, connect Pin 6 to 3.3v and Reset."));
            Serial.println(F("   Make sure that the bond on the phone/PC is deleted as well."));
            //We must have lost power and restarted and must restore the bonding infromation using the ACI Write Dynamic Data
            if (ACI_STATUS_TRANSACTION_COMPLETE == bond_data_restore(&aci_state, eeprom_status, &bonded_first_time))
            {
              Serial.println(F("Bond restored successfully"));
            }
            else
            {
              Serial.println(F("Bond restore failed. Delete the bond and try again."));
            }
          }
        }

        // Start bonding as all proximity devices need to be bonded to be usable
        if (ACI_BOND_STATUS_SUCCESS != aci_state.bonded)
        {
          lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
          Serial.println(F("No Bond present in EEPROM."));
          Serial.println(F("Advertising started : Waiting to be connected and bonded"));
        }
        else
        {
          //connect to an already bonded device
          //Use lib_aci_direct_connect for faster re-connections with PC, not recommended to use with iOS/OS X
          lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
          Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
        }
        break;
	Serial.print(F("Event OpCode = "));
	Serial.println(aci_evt->evt_opcode, HEX);
    }

	}
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
	
  }
  
  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}



void ble_begin()
{
		 Serial.begin(115200);

  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
  #if defined (__AVR_ATmega32U4__)
    while(!Serial)
    {}
    delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
  #elif defined(__PIC32MX__)
    delay(1000);
  #endif

  Serial.println(F("Arduino setup"));
  Serial.println(F("Set line ending to newline to send data from the serial monitor"));

  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*)setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /*
  Tell the ACI library, the MCU to nRF8001 pin connections.
  The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = reqn_pin; 
  aci_state.aci_pins.rdyn_pin   = rdyn_pin; 
  aci_state.aci_pins.mosi_pin   = mosi_pin;
  aci_state.aci_pins.miso_pin   = miso_pin;
  aci_state.aci_pins.sck_pin    = sck_pin;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed
  
  aci_state.aci_pins.reset_pin              = rst_pin; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = true; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 0;  
  
  
  lib_aci_init(&aci_state, false);

  
  digitalWrite(6, HIGH);  //Enable the pull-up resistor on button 3
  pinMode(6, INPUT); //Pin #6 on Arduino -> PAIRING CLEAR pin: Connect to 3.3v to clear the pairing
  if (0x01 == digitalRead(6))
  {
    //Clear the pairing
    Serial.println(F("Pairing cleared. Remove the wire on Pin 6 and reset the board for normal operation."));
    //Address. Value
    EEPROM.write(0, 0);
    while(1) {};
  }

  //Initialize the state of the bond
  aci_state.bonded = ACI_BOND_STATUS_FAILED;

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
}


#define MAX_TX_BUFF 64
static uint8_t tx_buff[MAX_TX_BUFF];
static uint8_t tx_buffer_len = 0;

bool ble_write(unsigned char data, uint8_t PIPE_NUMBER)
{
	bool writeStatus = false;
	
    if (lib_aci_is_pipe_available(&aci_state, PIPE_NUMBER) && (aci_state.data_credit_available > 0))
	{
		writeStatus = lib_aci_send_data(PIPE_NUMBER, &data, 1);
		if(writeStatus){
		aci_state.data_credit_available--;			
		}
	}
    return writeStatus;

}


bool bleCanSleep(){

	Serial.print(F("BLE Can Sleep Output = "));
	Serial.println(gotCMDRSP||gotData);
	if(gotCMDRSP||gotData)
	{
		return true;
	}
	else
		return false;
}

void bleWakeUp(){
	gotCMDRSP = 0;
	gotData = 0;
}
