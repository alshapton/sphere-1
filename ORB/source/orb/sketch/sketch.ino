//
//           ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄
//          ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░▌
//          ▐░█▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀█░▌
//          ▐░▌       ▐░▌▐░▌       ▐░▌▐░▌       ▐░▌
//          ▐░▌       ▐░▌▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄█░▌
//          ▐░▌       ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░▌
//          ▐░▌       ▐░▌▐░█▀▀▀▀█░█▀▀ ▐░█▀▀▀▀▀▀▀█░▌
//          ▐░▌       ▐░▌▐░▌     ▐░▌  ▐░▌       ▐░▌
//          ▐░█▄▄▄▄▄▄▄█░▌▐░▌      ▐░▌ ▐░█▄▄▄▄▄▄▄█░▌
//          ▐░░░░░░░░░░░▌▐░▌       ▐░▌▐░░░░░░░░░░▌
//           ▀▀▀▀▀▀▀▀▀▀▀  ▀         ▀  ▀▀▀▀▀▀▀▀▀▀
//

//          Project ORB
//.                      "Optional ROM Board"
//
//          An Arduino-based emulator for the
//          4 x Intel 1702A TTL EPROM chips
//          and the "OS" for the SPHERE-1 Computer
//
//          Author:       Andrew Shapton
//
//          Credits:      Ben Zott for the ROM image binary
//
//          Revision:     0.1         Initial Release - no board fab
//                        0.2         Reduced number of pins for chip select by constructing a 4:2 Binary Encoder



// Program Size Information
// Status       sketch    %         Global Vars   %
// Baseline     7088      23%       298           14%
// DEBUG 1      7088      23%       298           14%
// DEBUG 0      5826      18%        83            4%

// Define DEBUG settings
#define DEBUG 0

#if DEBUG == 1
#define debug(txt) Serial.println(txt);
#define debugbegin() Serial.begin(9600); // Any baud rate should work
#else
#define debug(txt) 
#define debugbegin()
#endif 

// Include PDS V3N ROM image
#include "pds-v3n.h"

// Include functions
#include "functions.h"

// Include pin layouts for interface to Arduino Nano
#include "pinlayouts.h"

// Constants
constexpr int CHIP_SIZE = 256;            // Size of 1702A EPROM chip in bytes

constexpr int numberOfAddressPins = 8;    // Number of pins on the address bus
constexpr int numberOfChipSelectPins = 2; // Number of pins on the CS line (Chip Select)
constexpr int numberOfDataPins = 8;       // Number of pins on the data bus

String binaryString = "";                 // Empty string to store a string representation of a binary number
char char_array[numberOfAddressPins] = {};// Character array to store address from the address bus
int val[8] = {};                          // General purpose array to store 8 bits
int loc;                                  // Location within a ROM image   
int value;                                // value of a specified location in a ROM image
int cs = -1;                              // initialise current CS to -1 - indicating no chip has been selected, so locate ROM

void setup() {
  debugbegin()

  // Initialise the Chip Select pins
  for (int thisPin = 0; thisPin < numberOfChipSelectPins; thisPin++) {
    pinMode(chipSelectPin[thisPin], INPUT_PULLUP);    // sets the digital pin as input with a PULLUP
  } 
    // Initialise the address bus pins
  for (int thisPin = 0; thisPin < numberOfAddressPins; thisPin++) {
    pinMode(addressPin[thisPin], INPUT_PULLUP);    // sets the digital pin as input with a PULLUP
  }

  // Write a LOW signal to the Data Lines (thus initialising them to an analogue read status)
  for (int thisPin = 0; thisPin < numberOfDataPins; thisPin++) {
    pinMode(dataPin[thisPin], OUTPUT);    // sets the digital pin as output 
    analogWrite(dataPin[thisPin], 255);
  }
}

void loop() {

  // Acquire the selected chip 
  cs = chipselectBC(chipSelectPin);
  if (cs > 0)
    {
      // If there is a valid chip selected (there should never not be)
      for (int thisVal = 0; thisVal < numberOfAddressPins; thisVal++) {
        val[thisVal] = 0;
      }

      // initialise the value of a string to null to receive the binary address from the address bus
      binaryString = "";
      for (int readAddressPin = 0; readAddressPin < numberOfAddressPins; readAddressPin++)
      {
        val[readAddressPin] = not(digitalRead(addressPin[readAddressPin]));   // read this input pin
        String pin = String(val[readAddressPin]);
        binaryString = binaryString + pin;
      }

    // Terminate the string
    binaryString = binaryString + "\0";
  
    // Convert the binary string to a character array, and then on to a decimal address
    binaryString.toCharArray(char_array, numberOfAddressPins+1);
    loc = bin2dec(char_array);

    // Retrieve value from ROM image
    int address = loc + (((cs -1) * CHIP_SIZE));    // Ensure the selected chip is taken into consideration
    value = pgm_read_byte_near( &(ROM[address]) );  // Note that the ROM image is stored in program memory
    
    // Left pad with zeroes to length of data bus
    String valueBinary = (padleft(String(value, BIN),numberOfDataPins,"0"));

    // Debug only
    debug("CS " + String(cs) + " / LOC(" + loc + "/" + binaryString + " ) / ABS " + String(address) + " / VAL(" + value + "/" + valueBinary + ")");
    
    // Write an output signal to each of the 8 data lines
    for (int thisPin = 0; thisPin < numberOfDataPins; thisPin++) 
      {
        char thisBin = valueBinary.charAt(thisPin);
        if (thisBin == '1')
          {      
            analogWrite(dataPin[thisPin], 255);
          }
        else
          {
            analogWrite(dataPin[thisPin], 0);
          }
      }
    }
}

