/*
 * Text-mode driver for ADS129x 
 * for Arduino Due
 *
 * Copyright (c) 2013 by Adam Feuer <adam@adamfeuer.com>
 * Copyright (c) 2012 by Chris Rorden
 * Copyright (c) 2012 by Steven Cogswell and Stefan Rado 
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// https://github.com/ivanseidel/DueTimer
#include <DueTimer.h> // because TimeOne does not work with Due
#include <util.h> // for htonl, required Ethernet2 library to be installed
#include <SPI.h>
#include <stdlib.h>
#include "adsCommand.h"
#include "ads129x.h"
#include "SerialCommand.h"
#include "Base64.h"
#include "SpiDma.h"

#define BAUD_RATE  115200     // WiredSerial ignores this and uses the maximum rate
#define txActiveChannelsOnly  // reduce bandwidth: only send data for active data channels
#define WiredSerial SerialUSB // use Due's Native USB port

int maxChannels = 0;
int numActiveChannels = 0;
boolean gActiveChan[9]; // reports whether channels 1..9 are active
boolean isRdatac = false;
boolean base64Mode = true;
boolean go_into_error_mode = false;

char hexDigits[] = "0123456789ABCDEF";

uint32_t packets_rxd = 0;  // drdys from the ADS1299
volatile bool camera_exposure_start = false;
#pragma pack(push)
#pragma pack(1)

#define NUMBER_OF_CHANNELS_TO_COLLECT_FROM_ADC 8
struct Packet { 
    uint32_t packets_rxd_n; // network byte order  
    uint8_t  sample[3 + 3*NUMBER_OF_CHANNELS_TO_COLLECT_FROM_ADC];
    volatile uint8_t camera_exposure_start;// 1 channels (incl status) * 3 bytes/per
};

#define SAMPLES_TO_SEND_IN_ONE_PACKET 6


// send multiple packets at once to improve payload efficiency
struct Packet packets[SAMPLES_TO_SEND_IN_ONE_PACKET] = {0};
#pragma pack(pop)

int packet_index = SAMPLES_TO_SEND_IN_ONE_PACKET-1;
// http://stackoverflow.com/questions/4715415/base64-what-is-the-worst-possible-increase-in-space-usage
// (sizeof(packets)+3)/3 * 4

char serial_buf[2000] = {0}; // 10 times larger than neccessary for base64 overhead

uint32_t millis_at_last_ping;
uint32_t LED_off_timeout_ms = 1000; // default to 1 second

const char *hardwareType = "unknown";
const char *boardName = "HackEEG";
const char *makerName = "StarCat, LLC";
const char *driverVersion = "v18.01.25";

int ADS1299_data_ready_to_be_collected = 0;

SerialCommand serialCommand;

SPISettings settingsA(4000000, MSBFIRST, SPI_MODE1);

//depends on hardware connections
int framerate_divisor_to_multiplex_channel[8] = {3,0,4,6,7,5,1,2};

void setup() {  
  // Setup callbacks for SerialCommand commands 
  serialCommand.addCommand("version", version_command);        // Echos the driver version number
  serialCommand.addCommand("ping", ping_command);
  serialCommand.addCommand("set_timeout", set_LED_timeout_command);
  serialCommand.addCommand("get_timeout", get_LED_timeout_command);
  serialCommand.addCommand("error_test", error_test); 
  serialCommand.addCommand("status",status_command);        // Echos the driver version number
  serialCommand.addCommand("serialnumber",serialNumber_command);        // Echos the driver version number
  serialCommand.addCommand("ledon",ledOn_command);            // Turns Due onboad LED on
  serialCommand.addCommand("ledoff", ledOff_command);         // Turns Due onboard LED off
  serialCommand.addCommand("boardledoff", boardLedOff_command);  // Turns HackEEG GPIO LED off
  serialCommand.addCommand("boardledon", boardLedOn_command);    // Turns HackEEG GPIO LED on
  serialCommand.addCommand("wakeup", wakeup_command);         // Enter read data continuous mode
  serialCommand.addCommand("standby", standby_command);         // Enter read data continuous mode
  serialCommand.addCommand("reset", reset_command);         // Enter read data continuous mode
  serialCommand.addCommand("start", start_command);         // Enter read data continuous mode
  serialCommand.addCommand("stop", stop_command);         // Enter read data continuous mode
  serialCommand.addCommand("rdatac", rdatac_command);         // Enter read data continuous mode
  serialCommand.addCommand("sdatac", sdatac_command);         // Stop read data continuous mode
  serialCommand.addCommand("rdata", rdata_command);           // Read one sample of data from each channel
  serialCommand.addCommand("rreg", readRegister_command);     // Read ADS129x register, argument in hex, print contents in hex
  serialCommand.addCommand("wreg", writeRegister_command);    // Write ADS129x register, arguments in hex
  serialCommand.addCommand("base64", base64ModeOn_command);   // rdata commands send base64 encoded data - default
  serialCommand.addCommand("hex", hexModeOn_command);         // rdata commands send hex encoded data
  serialCommand.addCommand("help", help_command);             // Print list of commands
  serialCommand.addCommand("framerate",set_framerate_command);// Set framerate

  serialCommand.setDefaultHandler(unrecognized);      // Handler for any command that isn't matched 
  WiredSerial.begin(115200);
  arduinoSetup();
  adsSetup();
}

// never exits, blinking LED to indicate error
void error_loop() {
    // never use noInterrupts() here as it can brick the Arduino
    // plus delay() required interrupts to be enabled
    // read from serial to avoid blocking the USB host
    while(1) {
        digitalWrite(PIN_LED, HIGH);  
        delay(250);              
        digitalWrite(PIN_LED, LOW);
        delay(250);
        serialCommand.readSerial(); // still respond for debugging
    }
}

void select_framerate(int divisor) {
   
   if ( divisor < 8 && divisor >= 0) {
        int multiplex_channel = framerate_divisor_to_multiplex_channel[divisor];
        int multiplex_A = multiplex_channel % 2;
        int multiplex_B = (multiplex_channel/2) % 2;
        int multiplex_C = (multiplex_channel/4) % 2;
        digitalWrite(PIN_MULTIPLEX_A, multiplex_A);
        digitalWrite(PIN_MULTIPLEX_B, multiplex_B);
        digitalWrite(PIN_MULTIPLEX_C, multiplex_C);
   }


}

// turns off LED if we don't get pings fast enough
void check_if_LED_should_be_off() {
    unsigned long t = millis();
    unsigned long diff = t - millis_at_last_ping; // handles rollover?
    if (diff > LED_off_timeout_ms) {
        digitalWrite(PIN_LED, LOW);
    }
}

void loop() {
    if (go_into_error_mode)
        error_loop();
    if (ADS1299_data_ready_to_be_collected)
        sendSamples();
    else
        serialCommand.readSerial();
}

long hexToLong(char *digits) {
  using namespace std;
  char *error;
  long n = strtol(digits, &error, 16);
  if ( *error != 0 ) { 
    return -1; // error
  } 
  else {
    return n;
  }
}

long strToDec(char *digits) {

  using namespace std;
  char *error;
  long n = strtol(digits, &error, 10);
  if ( *error != 0 ) { 
    return -1; // error
  } 
  else {
    return n;
  }
}

void outputHexByte(int value) {
  int clipped = value & 0xff;
  char charValue[3];
  sprintf(charValue, "%02X", clipped);
  WiredSerial.print(charValue);
}

void encodeHex(char* output, char* input, int inputLen) {
  register int count = 0;
  for (register int i=0; i < inputLen; i++) {
    register uint8_t lowNybble = input[i] & 0x0f;
    register uint8_t highNybble = input[i] >> 4;
    output[count++] = hexDigits[highNybble];
    output[count++] = hexDigits[lowNybble];
  }
  output[count] = 0;
}

void version_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println(driverVersion);
}

void ping_command() {
    if (isRdatac) {
        millis_at_last_ping = millis();
        digitalWrite(PIN_LED, HIGH);
    }
}

void set_LED_timeout_command() {
    char *arg1; 
    arg1 = serialCommand.next();   
    if (arg1 != NULL) {
        char *error;
        LED_off_timeout_ms = strtol(arg1, &error, 10);
        if  ( *error != 0 ) {
            WiredSerial.println("404 Error: invalid value");
        }
        else {
            WiredSerial.println("200 Ok");
        }
    }
    else {
        WiredSerial.println("404 Error: value argument missing");
    }
}

void get_LED_timeout_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println(LED_off_timeout_ms);
}

void error_test() {
    go_into_error_mode = true;
}

void status_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.print("Board name: ");   
  WiredSerial.println(boardName);   
  WiredSerial.print("Board maker: ");   
  WiredSerial.println(makerName);   
  WiredSerial.print("Hardware type: ");   
  WiredSerial.println(hardwareType);   
  WiredSerial.print("Max channels: "); 
  WiredSerial.println(maxChannels); 
  detectActiveChannels();
  WiredSerial.print("Number of active channels: "); 
  WiredSerial.println(numActiveChannels); 
  WiredSerial.print("Driver version: "); 
  WiredSerial.println(driverVersion); 
}

void serialNumber_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println("Not implemented yet. ");
}


void ledOn_command() {
  digitalWrite(PIN_LED,HIGH);  
  WiredSerial.println("200 Ok");
  WiredSerial.println("LED on"); 
}

void ledOff_command() {
  digitalWrite(PIN_LED,LOW);
  WiredSerial.println("200 Ok");
  WiredSerial.println("LED off"); 
}

void boardLedOn_command() {
  int state = adc_rreg(ADS129x::GPIO);
  state = state & 0xF7;
  state = state | 0x80;
  adc_wreg(ADS129x::GPIO, state);
  WiredSerial.println("200 Ok");
  WiredSerial.println("Board GPIO LED on"); 
}

void boardLedOff_command() {
  int state = adc_rreg(ADS129x::GPIO);
  state = state & 0x77;
  adc_wreg(ADS129x::GPIO, state);
  WiredSerial.println("200 Ok");
  WiredSerial.println("Board GPIO LED off"); 
}

void base64ModeOn_command() {
  base64Mode = true;
  WiredSerial.println("200 Ok");
  WiredSerial.println("Base64 mode on - rdata commands will send bas64 encoded data."); 
}

void hexModeOn_command() {
  base64Mode = false;
  WiredSerial.println("200 Ok");
  WiredSerial.println("Hex mode on - rdata commands will send hex encoded data"); 
}

void help_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println("Available commands: "); 
  serialCommand.printCommands();
}

void readRegister_command() {
  using namespace ADS129x; 
  char *arg1; 
  arg1 = serialCommand.next();   
  if (arg1 != NULL) {
    long registerNumber = hexToLong(arg1);
    if (registerNumber >= 0) {
      int result = adc_rreg(registerNumber);
      WiredSerial.println("200 Ok");
      WiredSerial.print(" (Read Register "); 
      outputHexByte(registerNumber); 
      WiredSerial.print(") "); 
      WiredSerial.println();               
      outputHexByte(result);      
    } 
    else {
      WiredSerial.println("402 Error: expected hexidecimal digits."); 
    }
  } 
  else {
    WiredSerial.println("403 Error: register argument missing."); 
  }
}

void writeRegister_command() {
  char *arg1, *arg2; 
  arg1 = serialCommand.next();   
  arg2 = serialCommand.next();  
  if (arg1 != NULL) {
    if (arg2 != NULL) { 
      long registerNumber = hexToLong(arg1);
      long registerValue = hexToLong(arg2);
      if (registerNumber >= 0 && registerValue >= 0) {
        adc_wreg(registerNumber, registerValue);        
        WiredSerial.println("200 Ok"); 
        WiredSerial.print(" (Write Register "); 
        outputHexByte(registerNumber); 
        WiredSerial.print(" "); 
        outputHexByte(registerValue);
        WiredSerial.print(") ");
        WiredSerial.println();
      } 
      else {
        WiredSerial.println("402 Error: expected hexidecimal digits."); 
      }
    } 
    else {
      WiredSerial.println("404 Error: value argument missing."); 
    }
  } 
  else {
    WiredSerial.println("403 Error: register argument missing."); 
  }
}

void wakeup_command() {
  using namespace ADS129x; 
  adc_send_command(WAKEUP);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Wakeup command sent.");
}

void standby_command() {
  using namespace ADS129x; 
  adc_send_command(STANDBY);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Standby command sent.");
}

void reset_command() {
  using namespace ADS129x; 
  adc_send_command(RESET);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Reset command sent.");
}

void start_command() {
  using namespace ADS129x; 
  adc_send_command(START);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Start command sent.");
}

void stop_command() {
  using namespace ADS129x; 
  adc_send_command(STOP);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Stop command sent.");
}

void rdata_command() {
  using namespace ADS129x; 
  while (digitalRead(IPIN_DRDY) == HIGH);
  adc_send_command_leave_cs_active(RDATA);
  WiredSerial.println("200 Ok ");
  sendSample();
}

void rdatac_command() {
  using namespace ADS129x; 
  detectActiveChannels();
  if (numActiveChannels > 0) { 
    isRdatac = true;
    packet_index = SAMPLES_TO_SEND_IN_ONE_PACKET - 1;
    adc_send_command(RDATAC);
    WiredSerial.println("200 Ok");
    WiredSerial.println("RDATAC mode on.");
    attachInterrupt(digitalPinToInterrupt(IPIN_DRDY), drdy_isr, FALLING);
  } else {
    WiredSerial.println("405 Error: no active channels.");
  }
}

void sdatac_command() {
  using namespace ADS129x;
  detachInterrupt(digitalPinToInterrupt(IPIN_DRDY));
  ADS1299_data_ready_to_be_collected = 0;
  isRdatac = false;
  adc_send_command(SDATAC);
  WiredSerial.println("200 Ok");
  WiredSerial.println("RDATAC mode off."); 
}

void set_framerate_command() {
    char* arg1;
    arg1 = serialCommand.next();
    if(arg1 != NULL) {
        select_framerate(strToDec(arg1));
    }
    WiredSerial.println("200 Ok");
    WiredSerial.print("Framerate divisor set to ");
    WiredSerial.println(arg1);
}



// This gets set as the default handler, and gets called when no other command matches. 
void unrecognized(const char *command) {
  WiredSerial.println("406 Error: Unrecognized command."); 
}

void detectActiveChannels() {  //set device into RDATAC (continous) mode -it will stream data
  if ((isRdatac) ||  (maxChannels < 1)) return; //we can not read registers when in RDATAC mode
  //Serial.println("Detect active channels: ");
  using namespace ADS129x; 
  numActiveChannels = 0;
  for (int i = 1; i <= maxChannels; i++) {
    delayMicroseconds(1); 
    int chSet = adc_rreg(CHnSET + i);
    gActiveChan[i] = ((chSet & 7) != SHORTED);
    if ( (chSet & 7) != SHORTED) numActiveChannels ++;   
  }
}

//#define testSignal //use this to determine if your software is accurately measuring full range 24-bit signed data -8388608..8388607
#ifdef testSignal
int testInc = 1;
int testPeriod = 100;
byte testMSB, testLSB; 
#endif 

inline void sendSamples(void) { 
  sendSample();
}

// Use SAM3X DMA
inline void sendSample(void) {
  digitalWrite(PIN_CS, LOW);
  register int numSerialBytes = 3 + 3*NUMBER_OF_CHANNELS_TO_COLLECT_FROM_ADC; // 24-bits header plus 24-bits per channel
  uint8_t returnCode = spiRec(packets[packet_index].sample, numSerialBytes);
  digitalWrite(PIN_CS, HIGH);
  ADS1299_data_ready_to_be_collected = 0;
  if (packet_index == SAMPLES_TO_SEND_IN_ONE_PACKET-1) {
      int num_bytes = 0;
      num_bytes = base64_encode(serial_buf, (char *) &packets, sizeof(packets));
      
      int availableForWriteNum = WiredSerial.availableForWrite();
      if (availableForWriteNum > num_bytes) {
          // can block - only call if enough buffer space not to block
  
          //WiredSerial.println("404 Error: condition 1");
          WiredSerial.println(serial_buf);
      }
      else {
          char msg[100];
          sprintf(msg,"404 Error: condition %d", availableForWriteNum);
          WiredSerial.println(msg);
          // haven't tested to see if it is possible to get to this code
          detachInterrupt(digitalPinToInterrupt(IPIN_DRDY));
          isRdatac = false;
          using namespace ADS129x;
          adc_send_command(SDATAC);
          digitalWrite(PIN_LED, HIGH);
      }
  }

  else {
  
      //WiredSerial.println("404 Error: condition 3");
  }
}

void adsSetup() { //default settings for ADS1298 and compatible chips
  using namespace ADS129x;
  // Send SDATAC Command (Stop Read Data Continuously mode)
  delay(1000); //pause to provide ads129n enough time to boot up...
  adc_send_command(SDATAC);
  delayMicroseconds(2);
  //delay(100); 
  int val = adc_rreg(ID) ;
  switch (val & B00011111 ) { //least significant bits reports channels
  case  B10000: //16
    hardwareType = "ADS1294";
    maxChannels = 4;
    break;
  case B10001: //17
    hardwareType = "ADS1296";
    maxChannels = 6; 
    break;
  case B10010: //18
    hardwareType = "ADS1298";
    maxChannels = 8; 
    break;
  case B11110: //30
    hardwareType = "ADS1299";
    maxChannels = 1; 
    break;
  default: 
    maxChannels = 0;
  }
  if (maxChannels == 0)
      error_loop();
  // All GPIO set to output 0x0000: (floating CMOS inputs can flicker on and off, creating noise)
  adc_wreg(GPIO, 0);
  adc_wreg(CONFIG3,PD_REFBUF | CONFIG3_const);
  //FOR RLD: Power up the internal reference and wait for it to settle
  // adc_wreg(CONFIG3, RLDREF_INT | PD_RLD | PD_REFBUF | VREF_4V | CONFIG3_const);
  // delay(150);
  // adc_wreg(RLD_SENSP, 0x01);	// only use channel IN1P and IN1N
  // adc_wreg(RLD_SENSN, 0x01);	// for the RLD Measurement
  adc_wreg(CONFIG1,HIGH_RES_500_SPS);
  adc_wreg(CONFIG2, INT_TEST);	// generate internal test signals
  // Set the first two channels to input signal
  for (int i = 1; i <= 1; ++i) {
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_1X); //report this channel with x12 gain
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_12X); //report this channel with x12 gain
    adc_wreg(CHnSET + i, TEST_SIGNAL | GAIN_12X); //create square wave
    //adc_wreg(CHnSET + i,SHORTED); //disable this channel
  }
  for (int i = 2; i <= 2; ++i) {
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_1X); //report this channel with x12 gain
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_12X); //report this channel with x12 gain
    adc_wreg(CHnSET + i, TEST_SIGNAL | GAIN_12X); //create square wave
    //adc_wreg(CHnSET + i,SHORTED); //disable this channel
  }
  for (int i = 3; i <= 6; ++i) {
    adc_wreg(CHnSET + i, SHORTED); //disable this channel
  }
  for (int i = 7; i <= 8; ++i) {
    adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_1X); //report this channel with x12 gain
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_12X); //report this channel with x12 gain
    //adc_wreg(CHnSET + i, TEST_SIGNAL | GAIN_12X); //create square wave
    //adc_wreg(CHnSET + i,SHORTED); //disable this channel
  }
  digitalWrite(PIN_START, HIGH);  
}

void drdy_isr(){
    if (isRdatac)
        packets_rxd += 1;
        packet_index++;
        if (packet_index == SAMPLES_TO_SEND_IN_ONE_PACKET)
            packet_index = 0;
        packets[packet_index].packets_rxd_n = htonl(packets_rxd);
        packets[packet_index].sample[0] = 0xB;
        packets[packet_index].sample[1] = 0xB;
        packets[packet_index].sample[2] = 0xA;
        packets[packet_index].sample[3] = 0xA;
        packets[packet_index].sample[4] = 0xD;
        packets[packet_index].sample[5] = 0xD;

        if (camera_exposure_start) {
            packets[packet_index].camera_exposure_start = 1;
        }
        else {
            packets[packet_index].camera_exposure_start = 0;
        }
        camera_exposure_start = false;
        ADS1299_data_ready_to_be_collected = 1;
}

void cameraTriggerISR() 
{
    camera_exposure_start = true;
}

void arduinoSetup(){
  using namespace ADS129x;
  pinMode(PIN_MULTIPLEX_A,OUTPUT);
  pinMode(PIN_MULTIPLEX_B,OUTPUT);
  pinMode(PIN_MULTIPLEX_C,OUTPUT);
  select_framerate(7); //initialize to lowest triggering frequency
  pinMode(PIN_LED,OUTPUT);      // Configure the onboard LED for output
  digitalWrite(PIN_LED,LOW);    // default to LED off
  //prepare pins to be outputs or inputs
  //pinMode(PIN_SCLK, OUTPUT); //optional - SPI library will do this for us
  //pinMode(PIN_DIN, OUTPUT); //optional - SPI library will do this for us
  //pinMode(PIN_DOUT, INPUT); //optional - SPI library will do this for us
  pinMode(PIN_CAM_GPO,INPUT_PULLUP);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_START, OUTPUT);
  pinMode(IPIN_DRDY, INPUT);
  pinMode(PIN_CLKSEL, OUTPUT);// *optional
  pinMode(IPIN_RESET, OUTPUT);// *optional
  //pinMode(IPIN_PWDN, OUTPUT);// *optional
  digitalWrite(PIN_CLKSEL, HIGH); // internal clock
  //start Serial Peripheral Interface
  SPI.begin();
  SPI.beginTransaction(settingsA);
  //Start ADS1298
  delay(500); //wait for the ads129n to be ready - it can take a while to charge caps
  digitalWrite(PIN_CLKSEL, HIGH);// *optional
  delay(10); // wait for oscillator to wake up  
  delay(1);
  digitalWrite(IPIN_PWDN, HIGH); // *optional - turn off power down mode
  digitalWrite(IPIN_RESET, HIGH);
  delay(1000);// *optional
  digitalWrite(IPIN_RESET, LOW);
  delay(1);// *optional
  digitalWrite(IPIN_RESET, HIGH);
  delay(1);  // *optional Wait for 18 tCLKs AKA 9 microseconds, we use 1 millisecond
  // check for LED every 1/4 of a second
  Timer.getAvailable().attachInterrupt(check_if_LED_should_be_off).start(250000);
  attachInterrupt(digitalPinToInterrupt(3),cameraTriggerISR,FALLING);
} //setup()





