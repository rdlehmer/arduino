
const char* const SW_VERSION = "2024-10-05 v0.7.3";

// Ron Lehmer
//
// For the Arduino Uno R3/Mega 2560
//
// Prerequisties
//
//  PCF8574 Library - version 2.3.6 minimum
//  MCP23017 Library - version 2.0.0 minimum
//  Watchdog Library - version 3.0.2 minimum

///
/// Global defines
///

//#define DBGLVL1
//#define DBGLVL2
//#define DBGLVLS

#define SD_SYSTEM
#define NETWORK_SYSTEM
#define KEYPAD_SYSTEM
#define TURNOUT_SYSTEM
#define SIGNAL_SYSTEM

//#define PCF8574_LOW_MEMORY

#define ADDRESS_NUMBER_TOGGLE_BOARDS 16
#define ADDRESS_NUMBER_SENSOR_BOARDS 17
#define ADDRESS_NUMBER_QUADTURNOUT_BOARDS 18
#define ADDRESS_NUMBER_INDICATOR_BOARDS 19
#define ADDRESS_NUMBER_SIGNAL_BOARDS 20

#define QUADTURNOUT_STATE_BASEADD 40
#define TURNOUT_STATE_BASEADD 56

#define QUADTURNOUT_BASEADD 64
#define TURNOUT_BASEADD 352
#define SIZE_OF_TURNOUT 8

#define QUADTURNOUT_SENSOR_BASEADD 192
#define SIZE_OF_QUADTURNOUT_SENSOR 10

#define SENSOR_BASEADD 416
#define SIZE_OF_SENSOR 8

#define SIGNAL_INPUT_BASEADD 544
#define SIZE_OF_SIGNAL_INPUT 10

#define SIGNAL_BASEADD 1300
#define SIZE_OF_SIGNAL 15

#define INDICATOR_BASEADD 928
#define SIZE_OF_INDICATOR 2

#define TRACKPOWER_BASEADD 960

#define TRACKMAP_BASEADD 1024

#define TIME_STEP 50
#define UPDATE_TIME 60000
///
/// Global Includes
///

#include <EEPROM.h>
#ifdef SD_SYSTEM
#include <Ethernet.h>
#include <SPI.h>
#include <SD.h>
#endif
#include <Wire.h>
#include <PCF8574.h>
#include <MCP23017.h>

#include <Watchdog.h>

///
/// Global variables
///

#ifdef TURNOUT_SYSTEM
PCF8574 toggle1(32);
PCF8574 toggle2(33);

MCP23017 turnoutA(36);
MCP23017 turnoutB(37);
MCP23017 turnoutC(38);
#endif

#ifdef SIGNAL_SYSTEM
PCF8574 blocksensor1(34);
PCF8574 blocksensor2(35);

MCP23017 signalA(38);
MCP23017 signalB(39);
#endif

#ifdef KEYPAD_SYSTEM
MCP23017 power(39);
#endif

#ifdef TURNOUT_SYSTEM
PCF8574 indicator1(56);
PCF8574 indicator2(57);
#endif

String consoleBuffer;
struct StringObject {
  char nameLabel[7];
};

struct SignalObject {
  byte signalNumber;
  char signalHead[7];
  char leadingSignalHead[7];
};

struct SignalInputObject {
  byte signalNumber;
  byte inputMode;
  byte inputIndex;
  char inputName[7];
};

struct QuadSensorObject {
  byte boardNumber;
  byte sensorChannel;
  char sensorName[7];
  byte sensorNumber;
};

struct TrackMapObject {
  byte turnout[16];
};

static unsigned long 	ulsPreviousTime = 0;
static long 			counts = 0;
static unsigned long 	ulsTempTime = 0;
static int 				isRunFirstTime = 1;
static int 				isTheFlasher = 0;

#ifdef NETWORK_SYSTEM

EthernetClient 			TheEthernetClient;
static long 			lsReconnectionTimer = 0;
static byte 			bsIsJmriRunning = 0;

String commandBuffer;
String receiveBuffer;

#endif

#ifdef KEYPAD_SYSTEM

String serial1ReceiveBuffer;

#endif

Watchdog watchdog;


///
/// Classes
///

#ifdef TURNOUT_SYSTEM

///
/// cmrs_toggle - instantiate a toggle switch 
///  _state - last observed state
///		0 - neutral
///		1 - throw
///     2 - close
///  set(byte) - sets state
///  get()     - returns state
///

class cmrs_toggle {
  public:
    cmrs_toggle() {
     _state = 0;
    }
    
    ~cmrs_toggle() {
    }
    
    void set(byte arg) {
      _state = arg;
    }
    
    byte get() {
      return _state;
    }
    
  private:
    byte _state;
    
};

///
/// CMRStoggles - collection of toggle switches
///  init()           - initializes boards defined
///  scan()           - reads inputs from Toggle boards and sets individual toggles
///  getToggles(byte) - get state of i-th toggle
///

class CMRStoggles {
  public:
    CMRStoggles() {
    }
    
    ~CMRStoggles() {
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_TOGGLE_BOARDS,temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0; i < 8; i++ ) {
          toggle1.pinMode(i, INPUT_PULLUP);
          if ( _boards > 1 ) toggle2.pinMode(i, INPUT_PULLUP);
        }
        toggle1.begin();
        if ( _boards > 1 ) toggle2.begin();
      }
    }
    
    void scan() {
      int i;
      byte tempa,tempb;
      for ( i = 0 ; i < 4*_boards ; i++ ) {
        if ( i < 4 ) {
          tempa = toggle1.digitalRead(2*i);
          tempb = toggle1.digitalRead(2*i+1);
        } else if ( i < 8 ) {
          tempa = toggle2.digitalRead(2*(i-4));
          tempb = toggle2.digitalRead(2*(i-4)+1);
        }
        if ( tempa == LOW ) {
          toggles[i].set(1);       
        }
        else if ( tempb == LOW ) {
          toggles[i].set(2);
        }
        else {
          toggles[i].set(0);
        }
        prevtoggles[i].set(toggles[i].get());
      }
    }
    
    int getToggle(int arg) {
      return toggles[arg].get();
    }
    
  private:
    cmrs_toggle toggles[8];
    cmrs_toggle prevtoggles[8];
    int _boards;
    
};

///
/// cmrs_turnout - turnout class
///  init() - set up hardware boards and read in saved state from memory
///  set(byte) - take active toggle commmand and determine if switch needs to change
///              and calls private methods to throw or close switch
///  getControl() - returns which toggle channel controls switch locally
///

class cmrs_turnout {
  public:
    cmrs_turnout() {
      _state = 0;
      _channel = 0;
    }
    
    ~cmrs_turnout() {
    }
    
    void init(byte i, byte j) {
      byte temp;
      _control_channel = j;
      _channel = i+1;
      EEPROM.get(QUADTURNOUT_STATE_BASEADD+i,temp);
      if (temp == 1) {
        if ( i < 4 ) {
          turnoutA.pinMode(2*i, OUTPUT, LOW);
          turnoutA.pinMode(2*i+1, OUTPUT, HIGH);
        }
        else if ( i < 8 ) {
          turnoutB.pinMode(2*(i-4), OUTPUT, LOW);
          turnoutB.pinMode(2*(i-4)+1, OUTPUT, HIGH);
        }
        else if ( i < 12 ) {
          turnoutC.pinMode(2*(i-8), OUTPUT, LOW);
          turnoutC.pinMode(2*(i-8)+1, OUTPUT, HIGH);
        }
        _state = 1;
      }
      else {
        if ( i < 4 ) {
          turnoutA.pinMode(2*i, OUTPUT, HIGH);
          turnoutA.pinMode(2*i+1, OUTPUT, LOW);
        }
        else if ( i < 8 ) {
          turnoutB.pinMode(2*(i-4), OUTPUT, HIGH);
          turnoutB.pinMode(2*(i-4)+1, OUTPUT, LOW);
        }
        else if ( i < 12 ) {
          turnoutC.pinMode(2*(i-8), OUTPUT, HIGH);
          turnoutC.pinMode(2*(i-8)+1, OUTPUT, LOW);
        }
        _state = 0;
      } 
    }
    
    void set(byte control_status) {
      byte _stx = 0;
#ifdef NETWORK_SYSTEM
      _stx = sethw(control_status);
      if ( _stx == 1 ) sendUpdate();
#endif
    }
    
    byte sethw(byte control_status) {
      byte ret_val = 0;
      if ( control_status != 0 ) {
        if (( _state == 0 ) && ( control_status == 1 )) {
          turnout_throw();
          ret_val = 1;
        }
        else if (( _state == 1 ) && ( control_status == 2 )) {
          turnout_close();
          ret_val = 1;
        }
      }
      return ret_val;
    }
    
    byte getControl() {
      return _control_channel;
    }
    
 #ifdef NETWORK_SYSTEM   
    void sendUpdate() {
      char tempstr[7];
      if (( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 )) {
        String tempStr;
        EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*(_channel-1)+1,tempstr);
        tempStr = String(tempstr);
        if ( tempStr.length() != 0 ) {
          tempStr = String("TURNOUT "+String(tempstr));
          if ( _state == 0 ) {
            tempStr = String(tempStr+" CLOSED");
          }
          else if ( _state == 1 ) {
            tempStr = String(tempStr+" THROWN");       
          }
          else {
            tempStr = String(tempStr+" UNKNOWN");       
          }
          Serial.print("Send: ");
          Serial.println(tempStr);
          if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
        }
      }
    }
#endif

  private:
    void turnout_throw() {
      _state = 1;
      byte _chn = _channel - 1;
      if ( _chn < 4 ) {
        turnoutA.digitalWrite(2*_chn, LOW);
        turnoutA.digitalWrite(2*_chn+1, HIGH);
      }
      else if ( _chn < 8 ) {
        turnoutB.digitalWrite(2*(_chn-4), LOW);
        turnoutB.digitalWrite(2*(_chn-4)+1, HIGH);
      }
      else if ( _chn < 12 ) {
        turnoutC.digitalWrite(2*(_chn-8), LOW);
        turnoutC.digitalWrite(2*(_chn-8)+1, HIGH);
      }
      EEPROM.update(QUADTURNOUT_STATE_BASEADD+_chn,_state);
    }

    void turnout_close() {
      _state = 0;
      byte _chn = _channel - 1;
      if ( _chn < 4 ) {
        turnoutA.digitalWrite(2*_chn, HIGH);
        turnoutA.digitalWrite(2*_chn+1, LOW);
      }
      else if ( _chn < 8 ) {
        turnoutB.digitalWrite(2*(_chn-4), HIGH);
        turnoutB.digitalWrite(2*(_chn-4)+1, LOW);
      }
      else if ( _chn < 12 ) {
        turnoutC.digitalWrite(2*(_chn-8), HIGH);
        turnoutC.digitalWrite(2*(_chn-8)+1, LOW);
      }
      EEPROM.update(QUADTURNOUT_STATE_BASEADD+_chn,_state);
    }
  
    byte _state;
    byte _channel;
    byte _control_channel;
};

///
/// CMRSquadSensor
///   init() - This initializes the pins on each turnout board that are connected as inputs
///   scan() - Reads the hardware and determines if any of the input pins have changes states
///            If any has changed state, then software state is updated and a JMRI message is sent
///   sendUpdate(sensor number) - Formats the JMRI message and sends it if the network is available
///   sendQuadSensorUpdate(index) - Sends a periodic update for the sensor number (index) - index
///			   can range from 0 to 119 so input is guarded to only active boards/channels
///

class CMRSquadSensors {
  public:
    CMRSquadSensors() {
    }
    
    ~CMRSquadSensors() {
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_QUADTURNOUT_BOARDS, temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0; i < 8; i++ ) {
          turnoutA.pinMode(i+8, INPUT_PULLUP);
          if ( _boards > 1 ) turnoutB.pinMode(i+8, INPUT_PULLUP);
          if ( _boards > 2 ) turnoutC.pinMode(i+8, INPUT_PULLUP);
        }
      }
    }
    
    void scan() {
      QuadSensorObject quadSensorObj;
      int i;
      for ( i = 0 ; i < 16 ; i++ ) {
        EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*i, quadSensorObj);
        if ( (quadSensorObj.boardNumber != 0) && (quadSensorObj.boardNumber <= _boards) 
             && (quadSensorObj.sensorChannel != 0) && (quadSensorObj.sensorChannel <= 8) 
             && (quadSensorObj.sensorNumber != 0) && (quadSensorObj.sensorNumber != 255)) {
          byte temp = 0;
          switch ( quadSensorObj.boardNumber ) {
            case 1: temp = turnoutA.digitalRead(quadSensorObj.sensorChannel+7);
              break;
            case 2: temp = turnoutB.digitalRead(quadSensorObj.sensorChannel+7);
              break;
            case 3: temp = turnoutC.digitalRead(quadSensorObj.sensorChannel+7);
              break;
 // sensorChannel definition changed for Rev C and beyond boards
          }
          if ( temp == LOW ) {
            quadSensor[i].set(1);
          } else {
            quadSensor[i].set(0);
          }
          if ( prevquadSensor[i].get() != quadSensor[i].get() )
            sendUpdate(i);
          prevquadSensor[i].set(quadSensor[i].get());
        }
      }
    }
    
    int getQuadSensor(int arg) {
      return quadSensor[arg].get();
    }
 
 #ifdef NETWORK_SYSTEM
    void sendUpdate(byte arg) {
      char tempstr[7];
      if ( ( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 ) ) {
        String tempStr;
        byte _bo;
        byte _ch;
        EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*arg,_bo);
        EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*arg+1,_ch);
        if (( _bo != 0 ) && ( _ch != 0 )) {
          EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*arg+2,tempstr);
          tempStr = String(tempstr);
          if ( tempStr.length() != 0 ) {
            tempStr = String("SENSOR "+tempStr);
            if ( getQuadSensor(arg) == 0 ) {
              tempStr = String(tempStr+" INACTIVE");
            }
            else if ( getQuadSensor(arg) == 1 ) {
              tempStr = String(tempStr+" ACTIVE");
            }
            else {
              tempStr = String(tempStr+" UNKNOWN");
            }
            Serial.print("Send: ");
            Serial.println(tempStr);
            if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
          }
        }
      }   
    }
    void sendQuadSensorsStatus(int arg) {
      if ( ( arg >= 0 ) && ( arg < 16 ) ) {
        sendUpdate(arg);
      }
    }
#endif   

  private:
    cmrs_toggle quadSensor[16];
    cmrs_toggle prevquadSensor[16];
    int _boards;
    
};



///
/// CMRSturnouts - collection of turnouts
///  init() - initialize turnout boards and channels
///  getControl(i) - returns the toggle channel for i-th turnout
///  setControl(i,j) - sends the state (j) for the i-th turnout
///  setRemote(i,j) - if a remote turnout command is received, set the hardware and then check
///  				  if a second (slaved) turnout needs to be thrown as well
///  sendTurnoutsStatus(index) - Sends a periodic update for the turnout number (index) - index
///			   can range from 0 to 119 so input is guarded to only active boards/channels
///  setSlavedControl(i,j) - find the i-th controlled switch and set it to state j
///

class CMRSturnouts {
  public:
    CMRSturnouts() {
    }
    
    ~CMRSturnouts() {
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_QUADTURNOUT_BOARDS, temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0 ; i < 4*_boards ; i++ ) {
          turnout[i].init(i, EEPROM.read(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i));
        }
      }
    }
    
    byte getControl(byte arg) {
      return turnout[arg].getControl();
    }
    
    void setControl(byte arg, byte val) {
      turnout[arg].set(val);
    }
    
    void setRemote(byte arg, byte val) {
      turnout[arg].sethw(val);
      setSlavedControl(arg, val);
    }
    
#ifdef NETWORK_SYSTEM
    void sendTurnoutsStatus(int arg) {
      if ( _boards > 0 ) {
        if (( arg >= 0 ) && ( arg < 4*_boards )) {
          turnout[arg].sendUpdate();
        }
      }
    }
#endif

  private:
    void setSlavedControl(byte arg, byte val) {
      int i;
      byte master;
      byte temp;
      master = getControl(arg);
      if ( master != 0 ) {
        for ( i = 0 ; i < 4*_boards ; i++ ) {
          temp = getControl(i);
          if (( i != arg ) && ( master == temp )) {
            setControl(i, val);
          }
        }
      }
    }
    
    int _boards;
    cmrs_turnout turnout[12];
    
};

///
/// cmrs_indicator - indicator
///  init(byte) - sets the channel number
///  set(byte)  - sets the hardware state of the indicator
///  get()      - returns the state of the indicator
///

class cmrs_indicator {
  public:
    cmrs_indicator() {
      _state = 0;
    }
    
    ~cmrs_indicator() {    
    }
    
    void init (byte arg) {
      _channel = arg;
    }
    
    void set(byte arg) {
      _state = arg;
      if ( _channel < 4 ) {
        switch ( _state ) {
          case 1 : indicator1.digitalWrite(2*_channel, LOW);
                 indicator1.digitalWrite(2*_channel+1, HIGH);
                 break;
          case 2 : indicator1.digitalWrite(2*_channel, HIGH);
                 indicator1.digitalWrite(2*_channel+1, LOW);
                 break;
          default : indicator1.digitalWrite(2*_channel, HIGH);
                 indicator1.digitalWrite(2*_channel+1, HIGH);
                 break; 
        }
      }
      else {
        switch ( _state ) {
          case 1 : indicator2.digitalWrite(2*(_channel-4), LOW);
                 indicator2.digitalWrite(2*(_channel-4)+1, HIGH);
                 break;
          case 2 : indicator2.digitalWrite(2*(_channel-4), HIGH);
                 indicator2.digitalWrite(2*(_channel-4)+1, LOW);
                 break;
          default : indicator2.digitalWrite(2*(_channel-4), HIGH);
                 indicator2.digitalWrite(2*(_channel-4)+1, HIGH);
                 break; 
        }      
      }
    }
    
    byte get() {
      return _state;
    }
    
  private:
    byte _state;
    byte _channel;
    
};

///
/// CMRSindicators - collection of indicators
///  init() - initialize hardware indicator boards and indicator channel numbers
///  set(byte,byte) - sets the i-th indicator with state j
///

class CMRSindicators {
  public:
    CMRSindicators() {    
    }
    
    ~CMRSindicators() {  
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_INDICATOR_BOARDS, temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0; i < 8; i++ ) {
          indicators[i].init(i);
          indicator1.pinMode(i, OUTPUT, HIGH);
          if ( _boards > 1 ) indicator2.pinMode(i, OUTPUT, HIGH);
        }
        indicator1.begin();
        if ( _boards > 1 ) indicator2.begin();
      }
    }
    
    void set(byte arg, byte val) {
      indicators[arg].set(val);
    }
    
  private:
    int _boards;
    cmrs_indicator indicators[8];
    
};

#endif


#ifdef KEYPAD_SYSTEM

///
/// Power System - Controls the 16 relay outputs to control yard track power
///   Outputs are pulled low to trigger the relay but MUST be set at high impedance to shut them off.
///   init() - sets up every pin as an output or input to drive the optoisolator inputs
///   set(i,j) - set the i-th relay; j == 1 turns them on
///   sendUpdate(i) - not used...
///

class CMRSpower {
  public:
    CMRSpower() {
    
    }
    
    ~CMRSpower() {
    
    }
    
    void init() {
      int track, pin;
      byte temp;
      for ( track = 1 ; track < 17 ; track++ ) {
        EEPROM.get(TRACKPOWER_BASEADD+(track-1), temp);
        power_status[track-1].set(temp);
        if ( (track % 2) == 0 ) {
          pin = 7 + track/2;
        }
        else {
          pin = track/2;
        }
        if ( temp == 1 ) {
          power.pinMode(pin, OUTPUT, LOW );
        }
        else {
          power.pinMode(pin, INPUT_PULLUP );  
        }
      }
    }
    
    void set(byte arg, byte val) {
      int pin;  
      if ( (arg % 2) == 0 ) {
        pin = 7 + arg/2;
      }
      else {
        pin = arg/2;
      }
      if ( val == 1 ) {
        power.pinMode(pin, OUTPUT, LOW );
        EEPROM.update(TRACKPOWER_BASEADD+(arg-1), byte(1));
        power_status[arg-1].set(1);
      }
      else {
        power.pinMode(pin, INPUT_PULLUP );
        EEPROM.update(TRACKPOWER_BASEADD+(arg-1), byte(0));
        power_status[arg-1].set(0);
      }
    }

#if 0    
    void sendUpdate(byte arg) {   // Not used now as of v0.6.6
      byte temp;
     if ( arg == 0 ) {
        char command[20] = "KBTRACK ";
        char ctemp[3];     
        EEPROM.get(959,temp);
        strcat(command,itoa(int(temp),ctemp,10));
        strcat(command, " SELECT\n");
        Serial1.write(command,strlen(command));           
      }
      else if ( arg <= 10 ) {
        char command[20] = "KBPOWER ";
        char ctemp[3];
        strcat(command,itoa(int(arg),ctemp,10));
        if ( power_status[arg-1].get() == 1 ) {
          strcat(command," ON\n");
        }
        else {
          strcat(command," OFF\n");
        }
        Serial.print("Serial1 send: ");
        Serial.println(command);
        Serial1.write(command,strlen(command));
      }
    }
#endif
  private:
    cmrs_toggle power_status[16];
};

#endif


#ifdef SIGNAL_SYSTEM

///
/// CMRSsensors - collection of toggle switches
///  init()           - initializes boards defined
///  scan()           - reads inputs from blocksensor boards and sets individual sensor toggle
///  getSensors(byte) - get state of i-th sensor
///  sendUpdate(byte) - send update message on the i-th sensor
///  sendSensorsUpdate(inex) - Sends a periodic update for the sensor number (index) - index
///			   can range from 0 to 119 so input is guarded to only active boards/channels
///

class CMRSsensors {
  public:
    CMRSsensors() {
    }
    
    ~CMRSsensors() {
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_SENSOR_BOARDS,temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0; i < 8; i++ ) {
          blocksensor1.pinMode(i, INPUT_PULLUP);
          if ( _boards > 1 ) blocksensor2.pinMode(i, INPUT_PULLUP);
        }
        blocksensor1.begin();
        if ( _boards > 1 ) blocksensor2.begin();
      }
    }
    
    void scan() {
      int i;
      byte temp;
      for ( i = 0 ; i < 4*_boards ; i++ ) {
        if ( i < 4 ) {
          temp = blocksensor1.digitalRead(i);
        } else if ( i < 8 ) {
          temp = blocksensor2.digitalRead(i-4);
        }
        if ( temp == LOW ) {
          blockSensors[i].set(1);
        } else {
          blockSensors[i].set(0);
        }
        if ( prevSensors[i].get() != blockSensors[i].get() ) {
          sendUpdate(i);
        }
        prevSensors[i].set(blockSensors[i].get());
      }
    }
    
    int getSensor(int arg) {
      return blockSensors[arg].get();
    }
 
 #ifdef NETWORK_SYSTEM
    void sendUpdate(byte arg) {
      char tempstr[7];
      if ( ( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 ) ) {
        String tempStr;
        byte _ch;
        EEPROM.get(SENSOR_BASEADD+SIZE_OF_SENSOR*arg, _ch);
        if ( _ch != 0 ) {
          EEPROM.get(SENSOR_BASEADD+SIZE_OF_SENSOR*arg+1,tempstr);
          tempStr = String(tempstr);
          if ( tempStr.length() != 0 ) {
            tempStr = String("SENSOR "+tempStr);
            if ( blockSensors[arg].get() == 0 ) {
              tempStr = String(tempStr+" INACTIVE");
            }
            else if ( blockSensors[arg].get() == 1 ) {
              tempStr = String(tempStr+" ACTIVE");
            }
            else {
              tempStr = String(tempStr+" UNKNOWN");
            }
            Serial.print("Send: ");
            Serial.println(tempStr);
            if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
          }
        }
      }   
    }
    
    void sendSensorsStatus(int arg) {
      if ( ( arg >= 0 ) && ( arg < 8 ) ) {
        sendUpdate(arg);
      }
    }
#endif   
   
  private:
    cmrs_toggle blockSensors[8];
    cmrs_toggle prevSensors[8];
    int _boards;
    
};

///
/// CMRSsignalInputs - list of inputs for the signal system
///  init()           - initializes the inputs as 0 
///  scan()           - not used
///  set(int,byte)   - set state of i-th sensor with value j
///  get(int) 		 - get state of i-th sensor
///

class CMRSsignalInputs {
  public:
    CMRSsignalInputs() {
    }
    
    ~CMRSsignalInputs() {
    }
    
    void init() {
      int i;
      for ( i = 0 ; i < 32 ; i++ ) {
        signalInputs[i].set(2);  // unknown
        prevInputs[i].set(2);  //unknown
      }
    }  

    void scan() {
 
    }
    
    byte get(int arg_i) {
      return signalInputs[arg_i].get();
    }
    
    void set(int arg_i, byte arg_val) {
      signalInputs[arg_i].set(arg_val);
#ifdef DBGLVLS
      Serial.print("SignalInputs ");
      Serial.print(arg_i);
      Serial.print(" = ");
      Serial.println(arg_val);
#endif
    }

#ifdef NETWORK_SYSTEM
    void sendSignalInputsStatus(int arg) {
      if (( arg >= 0 ) && ( arg < 32)) {
        if ( get(arg) == 2 ) {
          if ( ( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 ) ) {
            byte temp;
            EEPROM.get(SIGNAL_INPUT_BASEADD + SIZE_OF_SIGNAL_INPUT*arg+1, temp);
            if (( temp == 7 ) || ( temp == 8 )) {
              char tempstr[7];              
              String tempStr;
              EEPROM.get(SIGNAL_INPUT_BASEADD + SIZE_OF_SIGNAL_INPUT*arg+3, tempstr);
              tempStr = String(tempstr);
              if ( tempStr.length() != 0 ) {
                tempStr = String("SENSOR "+tempStr);
                tempStr = String(tempStr + " UNKNOWN");
                Serial.print("Send: ");
                Serial.println(tempStr);
                if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
              }
            }
          }
        }
      }
    }
#endif

    
  private:
    cmrs_toggle signalInputs[32];
    cmrs_toggle prevInputs[32];

};

///
/// cmrs_signal - signal
///  init(byte) - sets the channel number
///  set(byte)  - sets the hardware state of the indicator
///  get()      - returns the state of the indicator
///

class cmrs_signal {
  public:
    cmrs_signal() {
      _state = 0;
    }
    
    ~cmrs_signal() {    
    }
    
    void init (byte arg) {
      _state = 0;
      _previousState = 0;
      _leadingState = 0;
      _channel = arg;
    }
    
    void set(byte arg) {
      _state = arg;
      if ( _channel < 4 ) {
        if ( ( _state == 2 ) || (( _state == 3 ) && ( isTheFlasher == 0 )) ) {
          signalA.digitalWrite(3*_channel, LOW);
        }
        else {
          signalA.digitalWrite(3*_channel, HIGH);
        }
         if ( ( _state == 4 ) || (( _state == 5 ) && ( isTheFlasher == 0 )) ) {
          signalA.digitalWrite(3*_channel+1, LOW);
        }
        else {
          signalA.digitalWrite(3*_channel+1, HIGH);
        }
         if ( ( _state == 6 ) || (( _state == 7 ) && ( isTheFlasher == 0 )) ) {
          signalA.digitalWrite(3*_channel+2, LOW);
        }
        else {
          signalA.digitalWrite(3*_channel+2, HIGH);
        }      
      }
      else {
        if ( ( _state == 2 ) || (( _state == 3 ) && ( isTheFlasher == 0 )) ) {
          signalB.digitalWrite(3*(_channel - 4), LOW);
        }
        else {
          signalB.digitalWrite(3*(_channel - 4), HIGH);
        }
         if ( ( _state == 4 ) || (( _state == 5 ) && ( isTheFlasher == 0 )) ) {
          signalB.digitalWrite(3*(_channel - 4)+1, LOW);
        }
        else {
          signalB.digitalWrite(3*(_channel - 4)+1, HIGH);
        }
         if ( ( _state == 6 ) || (( _state == 7 ) && ( isTheFlasher == 0 )) ) {
          signalB.digitalWrite(3*(_channel - 4)+2, LOW);
        }
        else {
          signalB.digitalWrite(3*(_channel - 4)+2, HIGH);
        }       
      }
      if ( _state != _previousState ) {
#ifdef NETWORK_SYSTEM
        sendUpdate();
#endif
        _previousState = _state;
      }
    }
    
    void setLeadingState(byte _arg) {
      _leadingState = _arg;
    }
    
    byte get() {
      return _state;
    }
    
    byte getLeadingState() {
      return _leadingState;
    }
    
#ifdef NETWORK_SYSTEM   
    void sendUpdate() {
      char tempstr[7];
      if (( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 )) {
        String tempStr;
        EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*(_channel)+1,tempstr);
        tempStr = String(tempstr);
        if ( tempStr.length() != 0 ) {
          tempStr = String("SIGNALHEAD "+String(tempstr));
          switch ( _state ) {
            case 0 : 
            default :
              tempStr = String(tempStr + " UNKNOWN");
              break;
            case 1 : 
              tempStr = String(tempStr + " DARK");
              break;
            case 2 : 
              tempStr = String(tempStr + " GREEN");
              break;
            case 3 : 
              tempStr = String(tempStr + " FLASHGREEN");
              break;
            case 4 : 
              tempStr = String(tempStr + " YELLOW");
              break;
            case 5 : 
              tempStr = String(tempStr + " FLASHYELLOW");
              break;
            case 6 : 
              tempStr = String(tempStr + " RED");
              break;
            case 7 : 
              tempStr = String(tempStr + " FLASHRED");
              break;
            case 8 : 
              tempStr = String(tempStr + " LUNAR");
              break;
            case 9 : 
              tempStr = String(tempStr + " FLASHLUNAR");
              break;
          }
          Serial.print("Send: ");
          Serial.println(tempStr);
          if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
        }
      }
    }
    
    void sendLeadingUpdate() {
      char tempstr[7];
      if (( TheEthernetClient.connected() ) && ( bsIsJmriRunning == 1 )) {
        String tempStr;
        EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*(_channel)+8,tempstr);
        tempStr = String(tempstr);
        if ( tempStr.length() != 0 ) {
          tempStr = String("SIGNALHEAD "+String(tempstr));
          tempStr = String(tempStr + " UNKNOWN");
          Serial.print("Send: ");
          Serial.println(tempStr);
          if (TheEthernetClient.connected()) TheEthernetClient.println(tempStr);
        }
      }    
    }
#endif
    
  private:
    byte _state;
    byte _previousState;
    byte _leadingState;
    byte _channel;
    
};


///
/// CMRSsignals - signal interface
///  init()           - initializes the inputs as 0 
///  scan()           - not used
///  set(int,byte)   - set state of i-th signal with value j
///  get(int) 		 - get state of i-th signal
///
// signal settings
//  0 - Unknown
//  1 - Dark
//  2 - Green
//  3 - Flashing Green
//  4 - Yellow
//  5 - Flashing Yellow
//  6 - Red
//  7 - Flashing Red
//  8 - Lunar
//  9 - Flashing Lunar

class CMRSsignals {
  public:
    CMRSsignals() {
    }
    
    ~CMRSsignals() {
    }
    
    void init() {
      byte temp;
      int i;
      EEPROM.get(ADDRESS_NUMBER_SIGNAL_BOARDS, temp);
      _boards = int(temp);
      if ( _boards > 0 ) {
        for ( i = 0; i < 16; i++ ) {
          signalA.pinMode(i , OUTPUT, HIGH);
          if ( _boards > 1 ) signalB.pinMode(i, OUTPUT, HIGH);
        }
        for ( i = 0; i < 8; i++ ) {
          signal[i].init(i);
        }
//        signalA.begin();
//        if ( _boards > 1 ) signalB.begin();
      }
    }  

    byte get(int arg_i) {
      return signal[arg_i].get();
    }
    
    void set(int arg_i, byte arg_val) {
      signal[arg_i].set(arg_val);
    }
    
    byte getLeadingState(int arg_i) {
      return signal[arg_i].getLeadingState();
    }
    
    void setLeadingState(int arg_i, byte arg_val) {
      signal[arg_i].setLeadingState(arg_val);
    }
    
    void advanceSignal(int arg_i) {
      byte _leadingState;
      byte _newState;
      _leadingState = getLeadingState(arg_i);
 #if 0
      switch ( _leadingState ) {
        case 2 :  // Green
        case 3 :  // Flashing Green
        case 4 :  // Yellow
        case 5 :  // Flashing Yellow
          _newState = 2;  // set Green
          break;
        case 6 :  // Red
        case 7 :  // Flashing Red
          _newState = 4;  // set Yellow
          break;
        default :
          _newState = 2;  // other states - set Green for now
          break;         
      }
#endif
      if ( _leadingState < 6 ) {
        _newState = 2;
      }
      else if ( _leadingState < 8 ) {
        _newState = 4;
      }
      set(arg_i, _newState);
    }
    
#ifdef NETWORK_SYSTEM
    void sendSignalsStatus(int arg) {
      if ( _boards > 0 ) {
        if (( arg >= 0 ) && ( arg < 4*_boards )) {
          signal[arg].sendUpdate();
          if ( signal[arg].getLeadingState() == 0 ) {
            signal[arg].sendLeadingUpdate();
          }
        }
      }
    }
#endif

  private:
    cmrs_signal signal[16];
    int 		_boards;

};


#endif


///
/// Class instantation
///
#ifdef TURNOUT_SYSTEM
CMRStoggles     TheToggles;
CMRSturnouts    TheTurnouts;
CMRSindicators  TheIndicators;
CMRSquadSensors TheQuadSensors;
#endif

#ifdef KEYPAD_SYSTEM
CMRSpower		ThePowerSystem;
#endif

#ifdef SIGNAL_SYSTEM
CMRSsensors		TheSensors;
CMRSsignalInputs TheSignalInputs;
CMRSsignals	    TheSignals;
#endif


///
///  BEGIN SETUP
///

void setup() {
  Serial.begin(9600);
  eeprom_init(); 
  Serial.print("CMRS CP_2560 ");
  Serial.println(SW_VERSION);
#ifdef SD_SYSTEM
  Serial.println("Starting SD System...");
  Ethernet.init(10); // Arduino Ethernet board SS  
  sdCardManager();
#endif
  Serial.println("Starting I2C System...");
  Wire.setClock(100000);  // Slow mode 10kHz // Normal mode 100kHz
  Wire.begin();
  scan_i2c();

#ifdef TURNOUT_SYSTEM
  TheToggles.init();
  TheTurnouts.init();
  TheQuadSensors.init();
//  TheTurnouts.show();
  TheIndicators.init();
#endif

#ifdef KEYPAD_SYSTEM
  ThePowerSystem.init();
  Serial1.begin(9600);
#endif

#ifdef SIGNAL_SYSTEM
  TheSensors.init();
  TheSignalInputs.init();
  TheSignals.init();
#endif

}

///
///  BEGIN LOOP
///

void loop() {
  unsigned long currentTime = millis();
  if ( currentTime < ulsPreviousTime )
    ulsPreviousTime = currentTime; 
  if ( currentTime < ulsTempTime )
    ulsTempTime = currentTime;


#ifdef NETWORK_SYSTEM
  if (( isRunFirstTime == 1 ) && ( lsReconnectionTimer == 0 )) {
    connectServer();
    isRunFirstTime = 0;
    lsReconnectionTimer = 60000;
    watchdog.enable(Watchdog::TIMEOUT_1S);
  }
  if (TheEthernetClient.available()) {
    char c = TheEthernetClient.read();
    if ( c != 10 )
      receiveBuffer = String(receiveBuffer+String(c));
    if ( c == 10 ) {
      Serial.print("Receive: ");
      Serial.println(receiveBuffer);
      commandBuffer = receiveBuffer;
      receiveBuffer = String();
      processCommandBuffer();  // This is where we process incoming messages
    }
  }
#ifdef KEYPAD_SYSTEM
  if (Serial1.available()) {
    int c = Serial1.read();
    if ( c != 10 )
      serial1ReceiveBuffer = String(serial1ReceiveBuffer+String(c));
    if ( c == 10 ) {
      commandBuffer = String();
      Serial.print("Serial1 Receive: ");
      Serial.println(serial1ReceiveBuffer.c_str());
      int j = serial1ReceiveBuffer.length();
      for ( int k = 0 ; k < j/2 ; k++ ) {
        char t1 = (char)atoi((serial1ReceiveBuffer.substring(2*k,2*k+2)).c_str());
        commandBuffer = String(commandBuffer+String(t1));
      }
      Serial.println(commandBuffer);
      serial1ReceiveBuffer = String();
      processCommandBuffer();  // This is where we process incoming messages
    }
  }
#endif
    // if the server's disconnected, stop the TheEthernetClient:
  if ( (!TheEthernetClient.connected() ) && ( isRunFirstTime == 0 ) ) {
    Serial.println();
    Serial.println("disconnecting.");
    TheEthernetClient.stop();
    // do nothing:
    isRunFirstTime = 2;  //waiting for timer
    bsIsJmriRunning = 0;
    lsReconnectionTimer = 60000;
  }  
#endif
  
  if ( currentTime > ( ulsPreviousTime + TIME_STEP ) ) {
  
#ifdef NETWORK_SYSTEM  
    if ( ( lsReconnectionTimer > 0 ) && ( isRunFirstTime == 2 ) ) {
      lsReconnectionTimer = lsReconnectionTimer - TIME_STEP;
    }
    if ( lsReconnectionTimer <= 0 ) {
      lsReconnectionTimer = 0;
      isRunFirstTime = 1;
    }
#endif

#ifdef TURNOUT_SYSTEM
    TheToggles.scan();
    setTurnouts();
    setIndicators();
    TheQuadSensors.scan();
#endif
    
#ifdef SIGNAL_SYSTEM
    TheSensors.scan();
    setSignalInputs();
    TheSignalInputs.scan();
    set_signals();
#endif

    ulsPreviousTime += TIME_STEP;
  }

  if ( currentTime > ( ulsTempTime + 500 )) {
    ulsTempTime += 500;
    counts++;
    if ( isTheFlasher == 0 ) {
      isTheFlasher = 1;
    }
    else {
      isTheFlasher = 0;
    }
#ifdef TURNOUT_SYSTEM
    TheTurnouts.sendTurnoutsStatus((counts+80) % 120);
    TheQuadSensors.sendQuadSensorsStatus((counts+40) % 120);
#endif

#ifdef SIGNAL_SYSTEM
    TheSignalInputs.sendSignalInputsStatus((counts+10) % 120);
    TheSensors.sendSensorsStatus((counts+60) % 120);
    TheSignals.sendSignalsStatus((counts+100) % 120);
#endif

  }

  watchdog.reset();
}

#ifdef TURNOUT_SYSTEM
///
/// setTurnouts - scans through the turnouts 
///   loop through turnouts
///   get toggle number that controls turnout
///   If turnout is active ( control > 0 ) then
///     get toggle state
///     pass it turnout if state is other than neutral
///

void setTurnouts() {
  byte temp;
  int i;
  EEPROM.get(ADDRESS_NUMBER_QUADTURNOUT_BOARDS, temp);
  for ( i = 0 ; i < 4*temp ; i++ ) {
    byte _control, _state;
    _control = EEPROM.read(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i);

    if ( _control > 0 ) {
      _state = TheToggles.getToggle(_control-1);
      if ( _state != 0 ) {
        TheTurnouts.setControl(i, _state);
      }
    }
  } 
}

///
/// setIndicators - set indicators
///  scan for source of indicator information
///   directly from turnout state (first memory value) unless it is set to 0
///   then take state data from the j-th Sensor element
///

void setIndicators() {
  int i;
  byte _control[2];
  byte _boards,temp;
  EEPROM.get(ADDRESS_NUMBER_INDICATOR_BOARDS, _boards);
  for ( i = 0; i < 4*_boards; i++ ) {
    EEPROM.get(INDICATOR_BASEADD+2*i,_control);
    if ( _control[0] != 0 ) {
      EEPROM.get(QUADTURNOUT_STATE_BASEADD+_control[0]-1,temp);
      TheIndicators.set(i,temp+1);
    }
    else if ( _control[1] != 0 ) {
//    TBD - sensors
      temp = TheQuadSensors.getQuadSensor(_control[1]-1);
      TheIndicators.set(i,temp+1);
    }
    else {
      TheIndicators.set(i,0);
    }
  }
}
#endif

#ifdef KEYPAD_SYSTEM

//
// set_yard_turnouts
// Read in the 16 turnout settings to get into a support yeard track.
//  If value is set to 0, then send a 2 - close to the turnout
//  If value is set to 1, then send a 1 - throw the turnout
//  track = 0 means line all lead switches for the mainline
//

void set_yard_turnouts(byte track) {
  TrackMapObject _track_map;
  byte i;
  if ( track < 16 ) {
    EEPROM.get(TRACKMAP_BASEADD+track*16, _track_map);
    byte temp1;
    for ( i = 0 ; i < 12 ; i++ ) {   // only support 12 turnouts currently
      temp1 = 2 - _track_map.turnout[i];
      TheTurnouts.setControl(i,temp1); 
    }
    EEPROM.update(959,track);
  }
}

#endif

#ifdef SIGNAL_SYSTEM

//
// setSignalInputs
//  Scan the 32 Signal Input entries at set or unset those inputs that are driven by 
//  local varibles, such as turnout positions and sensors.
//
void setSignalInputs() {
  int i;
  byte temp[3];
  byte _state;
  for ( i = 0 ; i < 32 ; i++ ) {
    EEPROM.get(SIGNAL_INPUT_BASEADD+SIZE_OF_SIGNAL_INPUT*i, temp);
    if (( temp[0] != 0 ) && ( temp[1] < 5 )) {  //don't process inputs that are controlled by the network
      switch ( temp[1] ) {
        case 1 :	
                EEPROM.get(QUADTURNOUT_STATE_BASEADD+temp[2]-1, _state);
//                TheSignalInputs.set(temp[0], _state);
        		break;
        case 2 :
                EEPROM.get(QUADTURNOUT_STATE_BASEADD+temp[2]-1, _state);
                if ( _state == 1 ) {
                  _state = 0;
                }
                else {
                  _state = 1;
                }
//                TheSignalInputs.set(temp[0], _state);                
        		break;
        case 3 :
        		_state = TheSensors.getSensor(temp[2]-1);
//        		TheSignalInputs.set(temp[0], _state);
        		break;
        case 4 :
        		_state = TheQuadSensors.getQuadSensor(temp[2]-1);
//        		TheSignalInputs.set(temp[0], _state);
        		break;
      }
      TheSignalInputs.set(temp[0], _state);
    }
  }
}

#endif

//////////////////////////////////////////////////////////////////////
//
//  NETWORK SYSTEM
//
//////////////////////////////////////////////////////////////////////

#ifdef NETWORK_SYSTEM

byte connectServer() {
  byte ret_val;
  byte mac[6];
  byte ipAddr[4];   
  byte serveripAddr[4];

  static byte Efirst = 1;

  EEPROM.get(0, mac);
  EEPROM.get(8, ipAddr);
  EEPROM.get(12, serveripAddr);

  IPAddress ip(ipAddr);
  IPAddress server(serveripAddr);

  Ethernet.begin(mac, ip);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet Hardware Error");
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(500);
  }

  // give the Ethernet shield a second to initialize:
  if ( Efirst == 1 ) {
//    delay(1000);
    Efirst = 0;
  }
  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (ret_val = TheEthernetClient.connect(server, 2048)) {
    Serial.println("connected");
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
}

void processCommandBuffer() {
  int index1,index2;
  int i;
  char tempstr[7];
  byte temp;
  String cmdLabel, command, tempStr;
  
  index1 = commandBuffer.indexOf(" ");
  index2 = commandBuffer.indexOf(" ",index1+1);
  cmdLabel = commandBuffer.substring(index1+1, index2);
  command = commandBuffer.substring(index2+1);
#ifdef TURNOUT_SYSTEM
  EEPROM.get(ADDRESS_NUMBER_QUADTURNOUT_BOARDS, temp);
  if ( commandBuffer.startsWith("TURNOUT") ) {				// Receiving a remote turnout command
    for ( i = 0 ; i < 4*temp ; i ++ ) {
      EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
      tempStr = String(tempstr);
      tempStr.trim();
      if ( tempStr == cmdLabel ) {
        if ( command.startsWith("CLOSED") ) {
          TheTurnouts.setRemote(i,2);
        } else if ( command.startsWith("THROWN") ) {
          TheTurnouts.setRemote(i,1);
        }          
      }
    }
  } else if ( commandBuffer.startsWith("SENSOR") ) {		// Receiving a remote sensor input
#ifdef SIGNAL_SYSTEM
    SignalInputObject siobj;
    for ( i = 0 ; i < 32 ; i++ ) {
      EEPROM.get(SIGNAL_INPUT_BASEADD+SIZE_OF_SIGNAL_INPUT*i,siobj);
      if (( siobj.signalNumber != 0 ) && (( siobj.inputMode == 7 ) || ( siobj.inputMode == 8 ))) {
        tempStr = String(siobj.inputName);
        if ( tempStr == cmdLabel ) {
          if ( command.startsWith("ACTIVE") ) {
            TheSignalInputs.set(i, 8 - siobj.inputMode);  // 1 if mode 7 or 0 if mode 8
          } else if ( command.startsWith("INACTIVE") ) {
            TheSignalInputs.set(i, siobj.inputMode - 7);  // 0 if mode 7 or 1 if mode 8
          }
        }
      }
    }
#endif
  } else if ( commandBuffer.startsWith("SIGNALHEAD") ) {
#ifdef SIGNAL_SYSTEM
    SignalObject sobj;
    EEPROM.get(ADDRESS_NUMBER_SIGNAL_BOARDS, temp);
    for ( i = 0 ; i < 4*temp ; i++ ) {
      EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i,sobj);
      tempStr = String(sobj.leadingSignalHead);
      if ( tempStr == cmdLabel ) {
        TheSignals.setLeadingState(i,signalAspectToCode(command));
      }
    }  
#endif  
  } else if ( commandBuffer.startsWith("NODE jmri") ) {
    bsIsJmriRunning = 1;
    Serial.println("Connection to JMRI successful.");
  }
#endif
#ifdef KEYPAD_SYSTEM
  else if ( commandBuffer.startsWith("KBPOWER") ) {    // KBPOWER nn ON/OFF via Serial
    byte track1 = byte(cmdLabel.toInt());
    if ( command.startsWith("ON") ) {
      ThePowerSystem.set(track1,1);
    } else if ( command.startsWith("OFF") ) {
      ThePowerSystem.set(track1,0);
    }
  } else if ( commandBuffer.startsWith("KBTRACK") ) {  // KBTRACK nn SELECT via Serial
    byte track1 = byte(cmdLabel.toInt());
    set_yard_turnouts(track1);
  }
#endif
}

#endif

#ifdef SIGNAL_SYSTEM

byte signalAspectToCode(String arg) {
  byte rtn_val;
  if ( arg.startsWith("DARK") ) {
    rtn_val = 1;
  }
  else if ( arg.startsWith("RED") ) {
    rtn_val = 6;
  }
  else if ( arg.startsWith("FLASHRED") ) {
    rtn_val = 7;
  }
  else if ( arg.startsWith("YELLOW") ) {
    rtn_val = 4;
  }
  else if ( arg.startsWith("FLASHYELLOW") ) {
    rtn_val = 5;
  }
  else if ( arg.startsWith("GREEN") ) {
    rtn_val = 2;
  }
  else if ( arg.startsWith("FLASHGREEN") ) {
    rtn_val = 3;
  }
  else if ( arg.startsWith("LUNAR") ) {
    rtn_val = 8;
  }
  else if ( arg.startsWith("FLASHLUNAR") ) {
    rtn_val = 9;
  }
#if 0
  else if ( arg.startsWith("UNKNOWN") ) {
    rtn_val = 0;
  }
#endif
  else {
    rtn_val = 0;
  }
  return rtn_val;
}

void set_signals() {
  byte _boards;
  byte _signalNumber;
  byte  temp2;
  byte _occupancy;
  int i,j;

  EEPROM.get(ADDRESS_NUMBER_SIGNAL_BOARDS, _boards);
  for ( i = 0 ; i < 4*_boards ; i++ ) {
    EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i,_signalNumber);
    if ( _signalNumber != 0 ) {
      _occupancy = 0;
      for ( j = 0 ; j < 32 ; j++ ) {
        EEPROM.get(SIGNAL_INPUT_BASEADD+SIZE_OF_SIGNAL_INPUT*j,temp2);
        if ( temp2 == _signalNumber ) {
          if ( TheSignalInputs.get(j) == 1 ) {
            _occupancy = 1;
          }
        }
      }
      if ( _occupancy == 1 ) {
        TheSignals.set(i, 6);  // RED
      }
      else {
        TheSignals.advanceSignal(i);        
      }
    }
  }
}
// signal settings
//  0 - Unknown
//  1 - Dark
//  2 - Green
//  3 - Flashing Green
//  4 - Yellow
//  5 - Flashing Yellow
//  6 - Red
//  7 - Flashing Red
//  8 - Lunar
//  9 - Flashing Lunar
#endif


///////////////////////////////////////////////////////////////////////////////
//
// EEPROM Initialization
//
///////////////////////////////////////////////////////////////////////////////

void eeprom_init() {
  int i;
  byte temp;

  EEPROM.get(6,temp);
  if ( temp == 255 ) {
    EEPROM.update(6,byte(0));
  }

  for ( i = 16; i < 40 ; i++ ) {
    EEPROM.get(i,temp);
    if ( temp == 255 ) {
      EEPROM.update(i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 8*SIZE_OF_TURNOUT ; i++ ) {
    EEPROM.get(TURNOUT_BASEADD+i,temp);
    if ( temp == 255 ) {
        EEPROM.update(TURNOUT_BASEADD+i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 16*SIZE_OF_TURNOUT ; i++ ) {
    EEPROM.get(QUADTURNOUT_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(QUADTURNOUT_BASEADD+i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 16*SIZE_OF_QUADTURNOUT_SENSOR ; i++ ) {
    EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(QUADTURNOUT_SENSOR_BASEADD+i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 16*SIZE_OF_SENSOR ; i++ ) {
    EEPROM.get(SENSOR_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(SENSOR_BASEADD+i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 32*SIZE_OF_SIGNAL_INPUT ; i++ ) {
    EEPROM.get(SIGNAL_INPUT_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(SIGNAL_INPUT_BASEADD+i,byte(0));
    }
  }
  
  
  for ( i = 0 ; i < 16*SIZE_OF_SIGNAL ; i++ ) {
    EEPROM.get(SIGNAL_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(SIGNAL_BASEADD+i,byte(0));
    }
  }
  
  for ( i = 0 ; i < 16; i++ ) {
    EEPROM.get(INDICATOR_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(INDICATOR_BASEADD+i,byte(0));
    }
  }

  for ( i = 0 ; i < 16; i++ ) {
    EEPROM.get(TRACKPOWER_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(TRACKPOWER_BASEADD+i,byte(0));
    }
  }

  for ( i = 0 ; i < 256; i++ ) {
    EEPROM.get(TRACKMAP_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(TRACKMAP_BASEADD+i,byte(0));
    }
  }
 
}


#ifdef SD_SYSTEM

///////////////////////////////////////////////////////////////////////////////
//
// SD Card System
//
///////////////////////////////////////////////////////////////////////////////

void writeSDCard() {
  int index1,i;
  byte temp;
  String filename;
  File myFile;
  index1 = consoleBuffer.indexOf(" ");
  if ( index1 != -1 ) {
    filename = consoleBuffer.substring(index1+1);
    myFile = SD.open(filename.c_str(), FILE_WRITE);
    if ( myFile ) { 
      for ( i = 0; i < 2048; i++ ) {    // for Mega 2560 increase to 2048
        EEPROM.get(i, temp);
        myFile.print(i);
        myFile.print(" ");
        myFile.println(temp);
        if ( ( i % 64 ) == 63 ) {
          Serial.print("#");
        }
      }
      myFile.close();
      Serial.println();
    }
    else {
      Serial.print("Error: unable to open file ");
      Serial.print(filename.c_str());
      Serial.println(" for write.");
    }
  }
  else {
    Serial.println("Error: no filename");
  }
}

void readSDCard() {
  int index1,i;
  int memAdd;
  char temp;
  byte tempData,tempComp;
  String filename;
  String dataBuffer;
  File myFile;
  i = 0;
  index1 = consoleBuffer.indexOf(" ");
  if ( index1 != -1 ) {
    filename = consoleBuffer.substring(index1+1);
    myFile = SD.open(filename.c_str());
    if ( myFile ) {
      while ( myFile.available() ) {
        temp = myFile.read();
        if (( temp != 10 )) {
          dataBuffer = String(dataBuffer+String(temp));
        }
        else {
          index1 = dataBuffer.indexOf(" ");
          memAdd = (dataBuffer.substring(0,index1)).toInt();
          tempData = (dataBuffer.substring(index1+1)).toInt();
          EEPROM.update(memAdd,tempData);
          if ( ( i % 64 ) == 63 ) {
            Serial.print("#");
          }
          dataBuffer = String();
          i++;
        }
      }
// more to do here
      myFile.close();
      Serial.println();
    }
    else {
      Serial.println("Error: Unable to open file for read.");
    }
  }
  else {
    Serial.println("Error: no filename");
  }
}

void sdCardManager() {
  byte sdSystemDone = 0;
  if ( !SD.begin(4) ) {
    Serial.println("No SD card present. Starting...");
    return;
  }
  else {
    Serial.println("SD Card present. Starting console...(quit) to end.");
  }

  while ( sdSystemDone == 0 ) {
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      if ( inChar != 10 ) {
        consoleBuffer = String(consoleBuffer+String(inChar));
      }
      else {
        if ( consoleBuffer.startsWith("ex") ) {
          showExamine();
        }
        else if ( consoleBuffer.startsWith("dep") ) {
          depositData();
        }
        else if ( consoleBuffer.startsWith("conf") ) {
          show_configuration();
        }
        else if ( consoleBuffer.startsWith("write") ) {
          writeSDCard();
        }
        else if ( consoleBuffer.startsWith("read") ) {
          readSDCard();
        }
        else if ( consoleBuffer.startsWith("quit") ) {
          sdSystemDone = 1;
        }
        consoleBuffer = String();
      }
    }
  }
}

void show_configuration() {
  int i;
  byte contents;

//
// EEPROM memory map
//
// start  end     Function
//  000   005   MAC Address (hex)
  Serial.print("MAC Address: ");
  for ( i = 0 ; i < 6 ; i++ ) {
    EEPROM.get(i,contents);
    Serial.print(contents,HEX);
    if ( i < 5 ) Serial.print("-");
  }
  Serial.println();
//  006         Debug mode
  EEPROM.get(6,contents);
  Serial.print("Debug Mode ");
  Serial.println(contents);
//  007         undefined
//  008   011   IP address of Controller
  Serial.print("IP address: ");
  for ( i = 8 ; i < 12; i++ ) {
    EEPROM.get(i,contents);
    Serial.print(contents);
    if ( i < 11 ) Serial.print(".");
  }
  Serial.println();
//  012   015   IP address of JMRI SimpleServer (running port 2048)
  Serial.print("Server IP address: ");
  for ( i = 12 ; i < 16; i++ ) {
    EEPROM.get(i,contents);
    Serial.print(contents);
    if ( i < 15 ) Serial.print(".");
  }
  Serial.println();
//  016         Number of Toggle Boards starting with add 32 (32 - 33)
//  017         Number of Sensor Boards starting with add 34 (34 - 35)
//  018         Number of Quad Turnout Boaards starting with add 36 (36 - 39)
//  019         Number of Indicator Boards
//  020         Number of Signal Boards
//  021         Number of Dual Turnout Proto Boards starting with add 56
//  022         Number of Relay Boards (max 1) add 39
  Serial.print("# Toggles ");
  EEPROM.get(16,contents);
  Serial.print(contents);
  Serial.print(" # Sensors ");
  EEPROM.get(17,contents);
  Serial.print(contents);
  Serial.print(" # Quad ");
  EEPROM.get(18,contents);
  Serial.print(contents);
  Serial.print(" # Indic ");
  EEPROM.get(19,contents);
  Serial.print(contents);
  Serial.print(" # Signals ");
  EEPROM.get(20,contents);
  Serial.print(contents);
  Serial.print(" # Dual ");
  EEPROM.get(21,contents);
  Serial.println(contents);
  Serial.print(" # Relay ");
  EEPROM.get(22,contents);
  Serial.println(contents);
//  023   039   undefined
//  040   055   Turnout Positions 0 - 15 (Quads)
//  056   063   Turnout Positions 0 - 7 (Duals)
//  064   071   Turnout Quad 1/1 [ 0 - toggle number (0 is off) ; [1-7] Name ]
//  072   079   Turnout Quad 1/2
//  080   087   Turnout Quad 1/3
//  088   095   Turnout Quad 1/4
//  096   103   Turnout Quad 2/1
//  104   111   Turnout Quad 2/2
//  112   119   Turnout Quad 2/3
//  120   127   Turnout Quad 2/4
//  128   135   Turnout Quad 3/1
//  136   143   Turnout Quad 3/2
//  144   151   Turnout Quad 3/3
//  152   159   Turnout Quad 3/4
//  160   167   Turnout Quad 4/1
//  168   175   Turnout Quad 4/2
//  176   183   Turnout Quad 4/3
//  184   191   Turnout Quad 4/4
  EEPROM.get(18,contents);
  for ( i = 0; i < 4*contents ; i++ ) {
    byte val;
    StringObject val2;
    EEPROM.get(QUADTURNOUT_BASEADD + SIZE_OF_TURNOUT*i,val);
    if (( val > 0 ) && ( val < 32 )) {
      EEPROM.get(QUADTURNOUT_BASEADD + SIZE_OF_TURNOUT*i + 1, val2);
      Serial.print("Turnout ");
      Serial.print(int(i/4));
      Serial.print("/");
      Serial.print(i-int(i/4)*4);
      Serial.print(" - toggle ");
      Serial.print(val);
      Serial.print(" - name ");
      Serial.println(val2.nameLabel); 
    }
  }
//  192   201   Turnout Quad Sensor 1 [ 0 - board ( 0 is off, 1-4 ), 1 - sensor 
//                              ( 0 is off, 1-8, 1&2 with turnout 1, etc. ), [2-8] Name, 9 - Sensor num ]
//  202   211   Turnout Quad Sensor 2
//  212   221   Turnout Quad Sensor 3
//  222   231   Turnout Quad Sensor 4
//  232   241   Turnout Quad Sensor 5
//  242   251   Turnout Quad Sensor 6
//  252   261   Turnout Quad Sensor 7
//  262   271   Turnout Quad Sensor 8
//  272   281   Turnout Quad Sensor 9
//  282   291   Turnout Quad Sensor 10
//  292   301   Turnout Quad Sensor 11
//  302   311   Turnout Quad Sensor 12
//  312   321   Turnout Quad Sensor 13
//  322   331   Turnout Quad Sensor 14
//  332   341   Turnout Quad Sensor 15
//  342   351   Turnout Quad Sensor 16
  for ( i = 0 ; i < 4*contents ; i++ ) {
    QuadSensorObject sen;
    EEPROM.get( QUADTURNOUT_SENSOR_BASEADD + SIZE_OF_QUADTURNOUT_SENSOR*i, sen);
    if (( sen.sensorNumber > 0 ) && (sen.sensorNumber < 255)) {
      Serial.print("Board ");
      Serial.print(sen.boardNumber);
      Serial.print(" Ch ");
      Serial.print(sen.sensorChannel);
      Serial.print(" Name ");
      Serial.print(sen.sensorName);
      Serial.print(" No. ");
      Serial.println(sen.sensorNumber);
    }
  }
//  352   359   Turnout Dual 1/1 [ 0 - toggle number (0 is off) ; [1-7] Name ]
//  360   367   Turnout Dual 1/2
//  368   375   Turnout Dual 2/1
//  376   383   Turnout Dual 2/2
//  384   391   Turnout Dual 3/1
//  392   399   Turnout Dual 3/2
//  400   407   Turnout Dual 4/1
//  408   415   Turnout Dual 4/2
  EEPROM.get(21,contents);
  for ( i = 0; i < 2*contents ; i++ ) {
    byte val;
    StringObject val2;
    EEPROM.get(TURNOUT_BASEADD + SIZE_OF_TURNOUT*i,val);
    if (( val > 0 ) && ( val < 9 )) {
      EEPROM.get(TURNOUT_BASEADD + SIZE_OF_TURNOUT*i + 1, val2);
      Serial.print("Turnout ");
      Serial.print(int(i/2));
      Serial.print("/");
      Serial.print(i-int(i/2)*2);
      Serial.print(" - toggle ");
      Serial.print(val);
      Serial.print(" - name ");
      Serial.println(val2.nameLabel); 
    }
  }
//  416   423   Sensor 1 [ 0 - active if non-zero, [1-7] Name ]
//  424   431   Sensor 2
//  432   439   Sensor 3
//  440   447   Sensor 4
//  448   455   Sensor 5
//  456   463   Sensor 6
//  464   471   Sensor 7
//  472   479   Sensor 8
//  480   487   Sensor 9
//  488   495   Sensor 10
//  496   503   Sensor 11
//  504   511   Sensor 12
//  512   519   Sensor 13
//  520   527   Sensor 14
//  528   535   Sensor 15
//  536   543   Sensor 16
  EEPROM.get(17,contents);
  for ( i = 0; i < 4*contents ; i++ ) {
    byte val;
    StringObject val2;
    EEPROM.get(SENSOR_BASEADD + SIZE_OF_SENSOR*i,val);
    if (( val > 0 ) && ( val < 32 )) {
      EEPROM.get(SENSOR_BASEADD + SIZE_OF_SENSOR*i + 1, val2);
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" - number ");
      Serial.print(val);
      Serial.print(" - name ");
      Serial.println(val2.nameLabel); 
    }
  }
//  544   553  Signal Input 1 [ 0 - signal ( 0 is off ) ; 1 - mode ( 0 is off, 1 is local turnout,
//                                2 is local turnout (reversed), 3 is local sensor, 4 is local
//                                quad turnout sensor, 5 is remote turnout, 6 is remote turnout (reversed),
//                                  7 is remote sensor ) ; 2 - local index, [3-9] remote name ]
//  554   563   Signal Input 2
//  564   573   Signal Input 3
//  574   583   Signal Input 4
//  584   593   Signal Input 5
//  594   603   Signal Input 6
//  604   613   Signal Input 7
//  614   623   Signal Input 8
//  624   633   Signal Input 9
//  634   643   Signal Input 10
//  644   653   Signal Input 11
//  654   663   Signal Input 12
//  664   673   Signal Input 13
//  674   683   Signal Input 14
//  684   693   Signal Input 15
//  694   703   Signal Input 16
//  704   713   Signal Input 17
//  714   723   Signal Input 18
//  724   733   Signal Input 19
//  734   743   Signal Input 20
//  744   753   Signal Input 21
//  754   763   Signal Input 22
//  764   773   Signal Input 23
//  774   783   Signal Input 24
//  784   793   Signal Input 25
//  794   803   Signal Input 26
//  804   813   Signal Input 27
//  814   823   Signal Input 28
//  824   833   Signal Input 29
//  834   843   Signal Input 30
//  844   853   Signal Input 31
//  854   863   Signal Input 32

  for ( i = 0 ; i < 32 ; i++ ) {
    SignalInputObject sobj;
    EEPROM.get(SIGNAL_INPUT_BASEADD + SIZE_OF_SIGNAL_INPUT*i, sobj);
    if (( sobj.signalNumber != 0 ) && ( sobj.inputMode != 0 )) {
      Serial.print("Sig Input ");
      Serial.print(i);
      Serial.print(" signal ");
      Serial.print(sobj.signalNumber);
      Serial.print(" mode ");
      Serial.print(sobj.inputMode);
      Serial.print(" index ");
      Serial.print(sobj.inputIndex);
      if ( sobj.inputMode > 3 ) {
        Serial.print(" label ");
        Serial.print(sobj.inputName);
      }
      Serial.println();
    }
  }

//  928   929   Indicator 1/1  [ 0 - switch number ; 1 - sensor number (if switch number is 0 ]
//  930   931   Indicator 1/2
//  932   933   Indicator 1/3
//  934   935   Indicator 1/4
//  936   937   Indicator 2/1
//  938   939   Indicator 2/2
//  940   941   Indicator 2/3
//  942   943   Indicator 2/4
  EEPROM.get(19,contents);
  for ( i = 0 ; i < 4*contents ; i++ ) {
      byte val1, val2;
      EEPROM.get(928+(2*i),val1);
      EEPROM.get(929+(2*i),val2);
      Serial.print("Indicator ");
      Serial.print(int(i/4)+1);
      Serial.print("/");
      Serial.print(i-int(i/4)*4+1);
      Serial.print(" - switch ");
      Serial.print(val1);
      Serial.print(" - sensor ");
      Serial.println(val2);
  }
//  944   959   RESERVED
//  960   975   Track Power Saved State
//  976   1023  RESERVED
//
// 1300  1314   Signal 1/1 [ 0 - active ; [1-7] signalhead name ; 
//                            [8-14] leading remote signalhead name ]
// 1315  1329   Signal 1/2
// 1330  1344   Signal 1/3
// 1345  1359   Signal 1/4
// 1360  1374   Signal 2/1
// 1375  1389   Signal 2/2
// 1390  1404   Signal 2/3
// 1405  1419   Signal 2/4
// 1420  1434   Signal 3/1
// 1435  1449   Signal 3/2
// 1450  1464   Signal 3/4
// 1465  1479   Signal 4/1
// 1480  1494   Signal 4/2
// 1495  1509   Signal 4/3
// 1510  1524   Signal 4/4
  EEPROM.get(20,contents);
  for ( i = 0; i < 4*contents ; i++ ) {
    SignalObject sig;
    EEPROM.get(SIGNAL_BASEADD + SIZE_OF_SIGNAL*i,sig);
    if (1) {
      Serial.print("Signal ");
      Serial.print(int(i/4));
      Serial.print("/");
      Serial.print(i-int(i/4)*4);
      Serial.print(" - signal ");
      Serial.print(sig.signalNumber);
      Serial.print(" name ");
      Serial.print(sig.signalHead);
      Serial.print(" leading ");
      Serial.println(sig.leadingSignalHead);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
//
// Console Deposit of Data
//   dep aaaa xxxx      aaaa address (decimal) xxxx value (0-255)
//   dep/h aaaa hh      aaaa address (decimal) hh value (00-FF hex)
//   dep/s aaaa CCCCCCC aaaa starting address CCCCCCC up to 7 characters [up to 7 bytes]
//   dep/a aaaa xxx.xxx.xxx.xxx aaaa starting address xxx.xxx.xxx.xxx (decimal IP address) [4 bytes]
//   dep/m aaaa XX-XX-XX-XX-XX-XX aaaa starting address XX - hex MAC address [6 bytes]
//
///////////////////////////////////////////////////////////////////////////////
void depositData() {
 int index1,index2,baseAdd;
 index1 = consoleBuffer.indexOf(" ");
 index2 = consoleBuffer.indexOf(" ",index1+1);
 if ( ( index1 != -1 ) && ( index2 != -1 ) ) {
   baseAdd = (consoleBuffer.substring(index1,index2)).toInt();
   String tempstr = consoleBuffer.substring(index2+1);
   if ( consoleBuffer.startsWith("dep/h") ) {
     char tempChar[2];
     (consoleBuffer.substring(index2)).toCharArray(tempChar,2);
     byte dataContent = hexToByte(tempChar[0])*16 + hexToByte(tempChar[1]);
     EEPROM.update(baseAdd,dataContent);
   }
   else if ( consoleBuffer.startsWith("dep/m") ) {
     byte mactemp[6];
     byte tempchar[3];
     for (byte octet = 0; octet < 6; octet++) {
       if ( octet < 5 ) {
         int pos = tempstr.indexOf("-");
         if ( pos > 0) {
           tempstr.getBytes(tempchar,3);
           mactemp[octet] = hexToByte(tempchar[0])*16 + hexToByte(tempchar[1]);
           tempstr = tempstr.substring(pos+1);
         }
       } 
       else {
         tempstr.getBytes(tempchar,3);
         mactemp[octet] = hexToByte(tempchar[0])*16 + hexToByte(tempchar[1]);
       }
     }
     EEPROM.put(baseAdd,mactemp); 
   }
   else if ( consoleBuffer.startsWith("dep/a") ) {
     byte iptemp[4];
     for (byte octet = 0; octet < 4; octet++) {
       if ( octet < 3 ) {
         int pos = tempstr.indexOf(".");
         if ( pos > 0) {
           int temp = ((tempstr.substring(0,pos)).toInt());
           iptemp[octet] = (char)temp;
           tempstr = tempstr.substring(pos+1);
         }
       } 
       else {
         int temp = (tempstr.toInt());
         iptemp[octet] = (char)temp;
       }
     }
     EEPROM.put(baseAdd,iptemp);    
   }
   else if ( consoleBuffer.startsWith("dep/s") ) {
     char tempchar[7];
     StringObject tempObj;
     tempstr.toCharArray(tempchar,7);
     for ( int i = 0; i < 7; i++ ) {
       tempObj.nameLabel[i] = tempchar[i];
     }
     tempObj.nameLabel[7] = '\0';
     EEPROM.put(baseAdd,tempObj);
   }
   else {
     byte dataContent = byte( (consoleBuffer.substring(index2)).toInt() );
     EEPROM.update(baseAdd,dataContent);
   }
 }
}

///////////////////////////////////////////////////////////////////////////////
//
// Console Examination of EEPROM
// ex aaaa dddd    aaaa starting address (decimal) dddd number of bytes (decimal) output decimal
// ex/h aaaa dddd  aaaa starting address (decimal) dddd number of bytes (decimal) output hex
// if dddd is missing, one byte is shown
//
///////////////////////////////////////////////////////////////////////////////
void showExamine() {
  int index1,index2,baseAdd,bytesToShow,i;
  index1 = consoleBuffer.indexOf(" ");
  index2 = consoleBuffer.indexOf(" ",index1+1);
  if ( index1 != -1 ) {
    baseAdd = (consoleBuffer.substring(index1,index2)).toInt();
    if ( index2 == -1 ) {
      bytesToShow = 1;
    }
    else {
      bytesToShow = (consoleBuffer.substring(index2)).toInt();
    }
    byte contents;
    for ( i = 0 ; i < bytesToShow ; i++ ) {
      if ( ( i % 8 ) == 0 ) {
        Serial.print(baseAdd+i);
        Serial.print(" | ");
      }
      EEPROM.get(baseAdd+i,contents);
      if ( consoleBuffer.startsWith("ex/h") ) {
        Serial.print(contents,HEX);
      }
      else if ( consoleBuffer.startsWith("ex/s") ) {
        Serial.print(char(contents));
      }
      else {
        Serial.print(contents);
      }
      Serial.print(" ");
      if ( (( i % 8 ) == 7 ) ) {
        Serial.println();
      }
    }
    Serial.println();
  }
}

byte hexToByte(char arg_char) {
  if ( (arg_char >= '0') && (arg_char <= '9') )
    return (byte)(arg_char - 48);
  if ( (arg_char >= 'A') && (arg_char <= 'F') )
    return (byte)(arg_char - 55);
  return 0;
}

#endif

///////////////////////////////////////////////////////////////////////////////
//
// I2C Utility
//
///////////////////////////////////////////////////////////////////////////////
void scan_i2c() {
  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 8; address < 119; address++) { // limited to valid hardware addresses
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if ( error == 0 ) {
      Serial.print("I2C device found at ");
      if ( address < 16 )
        Serial.print("0");
      Serial.print(address);
      Serial.println(" !");
      nDevices++;
    }
    else if ( error == 4 ) {
      Serial.print("Unknown error at address 0x");
      if ( address < 16 )
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if ( nDevices == 0 )
    Serial.println("No I2C devices found\n");
  else
    Serial.println("I2C Scan Done\n");

  //delay(500); // remove wait to reduce time for turnouts to move.
}

//
// EEPROM memory map
//
// start  end     Function
//  000   005   MAC Address (hex)
//  006         Debug Level
//  007         undefined
//  008   011   IP address of Controller
//  012   015   IP address of JMRI SimpleServer (running port 2048)
//  016         Number of Toggle Boards starting with add 32 (32 - 33)
//  017         Number of Sensor Boards starting with add 34 (34 - 35)
//  018         Number of Quad Turnout Boaards starting with add 36 (36 - 39)
//  019         Number of Indicator Boards
//  020         Number of Signal Boards
//  021         Number of Dual Turnout Proto Boards starting with add 56
//  022         Number of Relay Boards (max 1) add 39
//  023   039   undefined
//  040   055   Turnout Positions 0 - 15 (Quads)
//  056   063   Turnout Positions 0 - 7 (Duals)
//  064   071   Turnout Quad 1/1 [ 0 - toggle number (0 is off) ; [1-7] Name ]
//  072   079   Turnout Quad 1/2
//  080   087   Turnout Quad 1/3
//  088   095   Turnout Quad 1/4
//  096   103   Turnout Quad 2/1
//  104   111   Turnout Quad 2/2
//  112   119   Turnout Quad 2/3
//  120   127   Turnout Quad 2/4
//  128   135   Turnout Quad 3/1
//  136   143   Turnout Quad 3/2
//  144   151   Turnout Quad 3/3
//  152   159   Turnout Quad 3/4
//  160   167   Turnout Quad 4/1
//  168   175   Turnout Quad 4/2
//  176   183   Turnout Quad 4/3
//  184   191   Turnout Quad 4/4
//  192   201   Turnout Quad Sensor 1 [ 0 - board ( 0 is off, 1-4 ), 1 - sensor 
//                              ( 0 is off, 1-8 ), [2-8] Name, 9 - Sensor num ]
//  202   211   Turnout Quad Sensor 2
//  212   221   Turnout Quad Sensor 3
//  222   231   Turnout Quad Sensor 4
//  232   241   Turnout Quad Sensor 5
//  242   251   Turnout Quad Sensor 6
//  252   261   Turnout Quad Sensor 7
//  262   271   Turnout Quad Sensor 8
//  272   281   Turnout Quad Sensor 9
//  282   291   Turnout Quad Sensor 10
//  292   301   Turnout Quad Sensor 11
//  302   311   Turnout Quad Sensor 12
//  312   321   Turnout Quad Sensor 13
//  322   331   Turnout Quad Sensor 14
//  332   341   Turnout Quad Sensor 15
//  342   351   Turnout Quad Sensor 16
//  352   359   Turnout Dual 1/1 [ 0 - toggle number (0 is off) ; [1-7] Name ]
//  360   367   Turnout Dual 1/2
//  368   375   Turnout Dual 2/1
//  376   383   Turnout Dual 2/2
//  384   391   Turnout Dual 3/1
//  392   399   Turnout Dual 3/2
//  400   407   Turnout Dual 4/1
//  408   415   Turnout Dual 4/2
//  416   423   Sensor 1 [ 0 - active if non-zero, [1-7] Name ]
//  424   431   Sensor 2
//  432   439   Sensor 3
//  440   447   Sensor 4
//  448   455   Sensor 5
//  456   463   Sensor 6
//  464   471   Sensor 7
//  472   479   Sensor 8
//  480   487   Sensor 9
//  488   495   Sensor 10
//  496   503   Sensor 11
//  504   511   Sensor 12
//  512   519   Sensor 13
//  520   527   Sensor 14
//  528   535   Sensor 15
//  536   543   Sensor 16

//  544   553	Signal Input 1 [ 0 - signal ( 0 is off ) ; 1 - mode ( 0 is off, 1 is local turnout,
//                                2 is local turnout (reversed), 3 is local sensor, 4 is local
//                                quad turnout sensor, 5 is remote turnout, 6 is remote turnout (reversed),
//                                  7 is remote sensor, 8 is invert remote sensor ) ; 2 - local index, [3-9] remote name ]
//  554   563   Signal Input 2
//  564	  573	  Signal Input 3
//  574   583   Signal Input 4
//  584   593   Signal Input 5
//  594   603   Signal Input 6
//  604   613   Signal Input 7
//  614   623   Signal Input 8
//  624   633   Signal Input 9
//  634   643   Signal Input 10
//  644   653   Signal Input 11
//  654   663   Signal Input 12
//  664   673   Signal Input 13
//  674   683   Signal Input 14
//  684   693   Signal Input 15
//  694   703   Signal Input 16
//  704   713   Signal Input 17
//  714   723   Signal Input 18
//  724   733   Signal Input 19
//  734   743   Signal Input 20
//  744   753   Signal Input 21
//  754   763   Signal Input 22
//  764   773   Signal Input 23
//  774   783   Signal Input 24
//  784   793   Signal Input 25
//  794   803   Signal Input 26
//  804   813   Signal Input 27
//  814   823   Signal Input 28
//  824   833   Signal Input 29
//  834   843   Signal Input 30
//  844   853   Signal Input 31
//  854   863   Signal Input 32
//  864   927	undefined
//  928   929   Indicator 1/1  [ 0 - switch number ; 1 - sensor number (if switch number is 0 ]
//  930   931   Indicator 1/2
//  932   933   Indicator 1/3
//  934   935   Indicator 1/4
//  936   937   Indicator 2/1
//  938   939   Indicator 2/2
//  940   941   Indicator 2/3
//  942   943   Indicator 2/4
//  944   959   RESERVED
//  960   975   Track Power Saved State
//  976  1023   RESERVED
// 1024	 1039   KBTRACK TRACK 0
// 1040  1055   KBTRACK TRACK 1
// 1056  1071   KBTRACK TRACK 2
// 1072  1087   KBTRACK TRACK 3
// 1088  1103   KBTRACK TRACK 4
// 1104  1119   KBTRACK TRACK 5
// 1120  1135   KBTRACK TRACK 6
// 1136  1151   KBTRACK TRACK 7
// 1152  1167   KBTRACK TRACK 8
// 1168  1183   KBTRACK TRACK 9
// 1184  1199   KBTRACK TRACK 10
// 1200  1215   KBTRACK TRACK 11
// 1216  1231   KBTRACK TRACK 12
// 1232  1247   KBTRACK TRACK 13
// 1248  1265   KBTRACK TRACK 14
// 1266  1281   KBTRACK TRACK 15
// 1282  1299   RESERVED
// 1300  1314   Signal 1/1 [ 0 - active ; [1-7] signalhead name ; 
//                            [8-14] leading remote signalhead name ]
// 1315  1329   Signal 1/2
// 1330  1344   Signal 1/3
// 1345  1359   Signal 1/4
// 1360  1374   Signal 2/1
// 1375  1389   Signal 2/2
// 1390  1404   Signal 2/3
// 1405  1419   Signal 2/4
// 1420  1434   Signal 3/1
// 1435  1449   Signal 3/2
// 1450  1464   Signal 3/4
// 1465  1479   Signal 4/1
// 1480  1494   Signal 4/2
// 1495  1509   Signal 4/3
// 1510  1524   Signal 4/4
