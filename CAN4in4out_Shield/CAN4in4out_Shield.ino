// CAN4in4out_Shield


/*
  Copyright (C) 2023 Martin Da Costa
  Including copyrights from CBUS_1in1out and Arduino CBUS Libraries

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation:

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
      CBUSSwitch  -- library access required by CBUS and CBUS Config
      CBUSLED     -- library access required by CBUS and CBUS Config
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map UNO:
// Digital pin 2          Interupt CAN
// Digital pin 3 (PWM)    Module LED 1
// Digital pin 4          CBUS Green LED
// Digital pin 5 (PWM)    Module LED 2
// Digital pin 6 (PWM)    Module LED 3
// Digital pin 7          CBUS Yellow LED
// Digital pin 8          CBUS Switch
// Digital pin 9 (PWM)    Module LED 4
// Digital pin 10 (SS)    CS    CAN
// Digital pin 11 (MOSI)  SI    CAN
// Digital pin 12 (MISO)  SO    CAN
// Digital pin 13 (SCK)   Sck   CAN

// Digital / Analog pin 0     Module Switch 1
// Digital / Analog pin 1     Module Switch 2
// Digital / Analog pin 2     Module Switch 3
// Digital / Analog pin 3     Module Switch 4
// Digital / Analog pin 4     Not Used
// Digital / Analog pin 5     Not Used
// Digital / Analog pin 5     Not Used
//////////////////////////////////////////////////////////////////////////

#define DEBUG 1  // set to 0 for no serial debug

#if DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// 3rd party libraries
#include <Streaming.h>
#include <Bounce2.h>

// CBUS library header files
#include <CBUS2515.h>    // CAN controller and CBUS class
#include <CBUSswitch.h>  // pushbutton switch
#include <CBUSLED.h>     // CBUS LEDs
#include <CBUSconfig.h>  // module configuration
#include <cbusdefs.h>    // MERG CBUS constants
#include <CBUSParams.h>

#include "LEDControl.h"  // Monitor LEDs

////////////DEFINE MODULE/////////////////////////////////////////////////

// module name
unsigned char mname[7] = { '4', 'i', 'n', '4', 'o', 'u', 't' };

// constants
const byte VER_MAJ = 2;     // code major version
const char VER_MIN = 'b';   // code minor version
const byte VER_BETA = 0;    // code beta sub-version
const byte MANUFACTURER = MANU_MERG; // for boards in development.
const byte MODULE_ID = 81;          // CBUS module type

const byte LED_GRN = 4;             // CBUS green SLiM LED pin
const byte LED_YLW = 7;             // CBUS yellow FLiM LED pin
const byte SWITCH0 = 8;             // CBUS push button switch pin

const unsigned long CAN_OSC_FREQ = 16000000UL;  // Oscillator frequency on the CAN Shield

//Module objects
const byte LED[] = {3, 5, 6, 9};     // LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = {A0, A1, A2, A3};  // Module Switch takes input to 0V.

const int NUM_LEDS = sizeof(LED) / sizeof(LED[0]);
const int NUM_SWITCHES = sizeof(SWITCH) / sizeof(SWITCH[0]);

// module objects
Bounce moduleSwitch[NUM_SWITCHES];  //  switch as input
LEDControl moduleLED[NUM_LEDS];     //  LED as output
byte switchState[NUM_SWITCHES];

//////////////////////////////////////////////////////////////////////////

//CBUS pins
const byte CAN_INT_PIN = 2;  // Only pin 2 and 3 support interrupts
const byte CAN_CS_PIN = 10;
//const byte CAN_SI_PIN = 11;  // Cannot be changed
//const byte CAN_SO_PIN = 12;  // Cannot be changed
//const byte CAN_SCK_PIN = 13;  // Cannot be changed

// CBUS objects
CBUSConfig module_config;       // configuration object
CBUS2515 CBUS(&module_config);  // CBUS object
CBUSLED ledGrn, ledYlw;             // two LED objects
CBUSSwitch pb_switch;               // switch object

// forward function declarations
void eventhandler(byte, CANFrame *);
void processSerialInput(void);
void printConfig(void);

///  setup CBUS - runs once at power on called from setup()
//
void setupCBUS() {
  // set config layout parameters
  module_config.EE_NVS_START = 10;
  module_config.EE_NUM_NVS = NUM_SWITCHES;
  module_config.EE_EVENTS_START = 50;
  module_config.EE_MAX_EVENTS = 64;
  module_config.EE_NUM_EVS = NUM_LEDS;
  module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

  // initialise and load configuration
  module_config.setEEPROMtype(EEPROM_INTERNAL);
  module_config.begin();

  Serial << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
  Serial << F(", NN = ") << module_config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(module_config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setManufacturerId(MANUFACTURER);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);  
  
  // set CBUS LED pins and assign to CBUS
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);
  CBUS.setLEDs(ledGrn, ledYlw);

  // initialise CBUS switch and assign to CBUS
  pb_switch.setPin(SWITCH0, LOW);
  pb_switch.run();
  CBUS.setSwitch(pb_switch);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  if (pb_switch.isPressed() && !module_config.FLiM) {
    Serial << F("> switch was pressed at startup in SLiM mode") << endl;
    module_config.resetModule(ledGrn, ledYlw, pb_switch);
  }

  // opportunity to set default NVs after module reset
  if (module_config.isResetFlagSet()) {
    Serial << F("> module has been reset") << endl;
    module_config.clearResetFlag();
  }

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);

  // set CBUS LEDs to indicate mode
  CBUS.indicateMode(module_config.FLiM);

  // configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(2, 1);                  // more buffers = more memory used, fewer = less
  CBUS.setOscFreq(CAN_OSC_FREQ);          // select the crystal frequency of the CAN module
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN);  // select pins for CAN bus CE and interrupt connections
  CBUS.begin();
}
//
///  setup Module - runs once at power on called from setup()
//

void setupModule() {
  // configure the module switches, active low
  for (int i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].attach(SWITCH[i], INPUT_PULLUP);
    moduleSwitch[i].interval(5);
    switchState[i] = false;
  }

  // configure the module LEDs
  for (byte i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].setPin(LED[i]);
  }

  Serial << "> Module has " << NUM_LEDS << " LEDs and " << NUM_SWITCHES << " switches." << endl;
}


void setup() {
  Serial.begin(115200);
  Serial << endl << F("> ** CBUS m in n out v1 ** ") << __FILE__ << endl;

  setupCBUS();
  setupModule();

  // end of setup
  DEBUG_PRINT(F("> ready"));
}


void loop() {
  // do CBUS message, switch and LED processing
  CBUS.process();

  // process console commands
  processSerialInput();

  // Run the LED code
  for (int i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();
  
}  // End of Loop

void processSwitches(void) {
  bool isSuccess = true;
  for (int i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].update();
    if (moduleSwitch[i].changed()) {
      byte nv = i + 1;
      byte nvval = module_config.readNV(nv);
      byte opCode;

      DEBUG_PRINT(F("> Button ") << i << F(" state change detected"));
      Serial << F(" NV = ") << nv << F(" NV Value = ") << nvval << endl;

      switch (nvval) {
        case 1:
          // ON and OFF
          opCode = (moduleSwitch[i].fell() ? OPC_ACON : OPC_ACOF);
          DEBUG_PRINT(F("> Button ") << i
                                     << (moduleSwitch[i].fell() ? F(" pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode));
          isSuccess = sendEvent(opCode, i);
          break;

        case 2:
          // Only ON
          if (moduleSwitch[i].fell()) {
            opCode = OPC_ACON;
            DEBUG_PRINT(F("> Button ") << i << F(" pressed, send 0x") << _HEX(OPC_ACON));
            isSuccess = sendEvent(opCode, i);
          }
          break;

        case 3:
          // Only OFF
          if (moduleSwitch[i].fell()) {
            opCode = OPC_ACOF;
            DEBUG_PRINT(F("> Button ") << i << F(" pressed, send 0x") << _HEX(OPC_ACOF));
            isSuccess = sendEvent(opCode, i);
          }
          break;

        case 4:
          // Toggle button
          if (moduleSwitch[i].fell()) {
            switchState[i] = !switchState[i];
            opCode = (switchState[i] ? OPC_ACON : OPC_ACOF);
            DEBUG_PRINT(F("> Button ") << i
                                       << (moduleSwitch[i].fell() ? F(" pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode));
            isSuccess = sendEvent(opCode, i);
          }

          break;

        default:
          DEBUG_PRINT(F("> Invalid NV value."));
          break;
      }
    }
  }
  if (!isSuccess) {
    DEBUG_PRINT(F("> One of the send message events failed"));
  }
}

// Send an event routine according to Module Switch
bool sendEvent(byte opCode, unsigned int eventNo) {
  CANFrame msg;
  msg.id = module_config.CANID;
  msg.len = 5;
  msg.data[0] = opCode;
  msg.data[1] = highByte(module_config.nodeNum);
  msg.data[2] = lowByte(module_config.nodeNum);
  msg.data[3] = highByte(eventNo);
  msg.data[4] = lowByte(eventNo);

  bool success = CBUS.sendMessage(&msg);
  if (success) {
    DEBUG_PRINT(F("> sent CBUS message with Event Number ") << eventNo);
  } else {
    DEBUG_PRINT(F("> error sending CBUS message"));
  }
  return success;
}

//
/// called from the CBUS library when a learned event is received
//
void eventhandler(byte index, CANFrame *msg) {
  byte opc = msg->data[0];

  DEBUG_PRINT(F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));
  DEBUG_PRINT(F("> event handler: length = ") << msg->len);

  unsigned int node_number = (msg->data[1] << 8) + msg->data[2];
  unsigned int event_number = (msg->data[3] << 8) + msg->data[4];
  DEBUG_PRINT(F("> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(F("> op_code = ") << opc);

  switch (opc) {
    case OPC_ACON:
    case OPC_ASON:
      for (byte i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 1;
        byte evval = module_config.getEventEVval(index, ev);

        switch (evval) {
          case 1:
            moduleLED[i].on();
            break;

          case 2:
            moduleLED[i].flash(500);
            break;

          case 3:
            moduleLED[i].flash(250);
            break;

          default:
            break;
        }
      }
      break;
      
    case OPC_ACOF:
    case OPC_ASOF:
      for (byte i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 1;
        byte evval = module_config.getEventEVval(index, ev);

        if (evval > 0) {
          moduleLED[i].off();
        }
      }
      break;
  }
}

//
/// print code version config details and copyright notice
//
void printConfig(void) {
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Martin Da Costa (MERG M6223) 2023") << endl;
  
}

//
/// command interpreter for serial console input
//

void processSerialInput(void) {
  byte uev = 0;
  char msgstr[32];

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {

      case 'n':
        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (module_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID << F(", node number = ") << module_config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':
        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << module_config.EE_MAX_EVENTS << F(" EVs per event = ") << module_config.EE_NUM_EVS << F(" bytes per event = ") << module_config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (module_config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * module_config.EE_BYTES_PER_EVENT) << F(" of ") << (module_config.EE_MAX_EVENTS * module_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl
               << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (module_config.EE_NUM_EVS); j++) {
          sprintf(msgstr, "EV%03d | ", j + 1);
          Serial << msgstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            sprintf(msgstr, "  %03d  | ", j);
            Serial << msgstr;

            // for each data byte of this event
            for (byte e = 0; e < (module_config.EE_NUM_EVS + 4); e++) {
              sprintf(msgstr, " 0x%02hx | ", module_config.readEEPROM(module_config.EE_EVENTS_START + (j * module_config.EE_BYTES_PER_EVENT) + e));
              Serial << msgstr;
            }

            sprintf(msgstr, "%4d |", module_config.getEvTableEntry(j));
            Serial << msgstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':
        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= module_config.EE_NUM_NVS; j++) {
          byte v = module_config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl
               << endl;

        break;

      // CAN bus status
      case 'c':
        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        module_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        module_config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << module_config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        CBUS.renegotiate();
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
