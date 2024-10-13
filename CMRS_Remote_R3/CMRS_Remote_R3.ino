
                                             // Version v0.6.6a
// Ron Lehmer   2023-12-30
//
// For the Arduino Uno R3/Mega 2560
//
// Prerequisties
//
//  PCF8574 Library - version 2.3.6 minimum
//  MCP23017 Library - version 2.0.0 minimum
///
/// Global defines
///

//#define DBGLVL1
//#define DBGLVL2

//#define SD_SYSTEM
//#define NETWORK_SYSTEM
//#define KEYPAD_SYSTEM
#define TURNOUT_SYSTEM

//#define PCF8574_LOW_MEMORY

#define ADDRESS_NUMBER_TOGGLE_BOARDS 16
#define ADDRESS_NUMBER_QUADTURNOUT_BOARDS 18
#define ADDRESS_NUMBER_INDICATOR_BOARDS 19

#define QUADTURNOUT_STATE_BASEADD 40
#define TURNOUT_STATE_BASEADD 56

#define QUADTURNOUT_BASEADD 64
#define TURNOUT_BASEADD 352
#define SIZE_OF_TURNOUT 8

#define QUADTURNOUT_SENSOR_BASEADD 192
#define SIZE_OF_QUADTURNOUT_SENSOR 10

#define SENSOR_BASEADD 416
#define SIZE_OF_SENSOR 8

#define SIGNAL_BASEADD 544
#define SIZE_OF_SIGNAL 24

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
#include <SoftwareSerial.h>
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
  byte sensorNumber;
  byte turnout1Number;
  byte turnout2Number;
  char signalHead[7];
  char leadingSignalHead[7];
  char remoteSensor[7];
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

static unsigned long prevTime = 0;
static long counts = 0;
static unsigned long tempTime = 0;
static int run_first_time = 1;

#ifdef NETWORK_SYSTEM

static long reconnect_timer = 0;
EthernetClient client;
static byte jmri_running = 0;
String commandBuffer;
String receiveBuffer;

#endif

#ifdef KEYPAD_SYSTEM

String serial1ReceiveBuffer;

#endif

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
#ifdef DBGLVL1
      Serial.print("INIT: Initializing ");
      Serial.print(_boards);
      Serial.println(" Toggle boards.");
#endif
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
#ifdef DBGLVL1
      Serial.print("INIT: Initializing ");
      Serial.print(_boards);
      Serial.println(" Indicator boards.");
#endif
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



///
/// Class instantation
///
#ifdef TURNOUT_SYSTEM
CMRStoggles     TheToggles;
//CMRSturnouts    TheTurnouts;
CMRSindicators  TheIndicators;
//CMRSquadSensors TheQuadSensors;
#endif

///
///  BEGIN SETUP
///

void setup() {
  Serial.begin(9600);
  eeprom_init(); 
  Serial.println("CMRS CP_2560 v0.6.6a 2023-12-30");
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
//  TheTurnouts.init();
//  TheQuadSensors.init();
//  TheTurnouts.show();
  TheIndicators.init();
#endif

}

///
///  BEGIN LOOP
///

void loop() {
  unsigned long currentTime = millis();
  if ( currentTime < prevTime )
    prevTime = currentTime; 
  if ( currentTime < tempTime )
    tempTime = currentTime;


#ifdef NETWORK_SYSTEM
  if (( run_first_time == 1 ) && ( reconnect_timer == 0 )) {
    connectServer();
    run_first_time = 0;
    reconnect_timer = 60000;
  }
  if (client.available()) {
    char c = client.read();
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
    // if the server's disconnected, stop the client:
  if ( (!client.connected() ) && ( run_first_time == 0 ) ) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    // do nothing:
    run_first_time = 2;  //waiting for timer
    jmri_running = 0;
    reconnect_timer = 60000;
  }  
#endif
  
  if ( currentTime > ( prevTime + TIME_STEP ) ) {
  
#ifdef NETWORK_SYSTEM  
    if ( ( reconnect_timer > 0 ) && ( run_first_time == 2 ) ) {
      reconnect_timer = reconnect_timer - TIME_STEP;
    }
    if ( reconnect_timer <= 0 ) {
      reconnect_timer = 0;
      run_first_time = 1;
    }
#endif

#ifdef TURNOUT_SYSTEM
    TheToggles.scan();
//    setTurnouts();
    setIndicators();
//    TheQuadSensors.scan();
#endif
    
    prevTime += TIME_STEP;
  }

#ifdef TURNOUT_SYSTEM
  if ( currentTime > ( tempTime + 1000 )) {
    tempTime += 1000;
    counts++;
#ifdef NETWORK_SYSTEM
    TheTurnouts.sendTurnoutsStatus((counts+40) % 60);
    TheQuadSensors.sendQuadSensorsStatus((counts+20) % 60);
#endif
  }
#endif
}

#ifdef TURNOUT_SYSTEM

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
#ifdef DBGLVL2
      Serial.print("Indicator: set channel ");
      Serial.print(i);
      Serial.print(" val ");
      Serial.println(temp+1);
#endif
    }
    else if ( _control[1] != 0 ) {
//    TBD - sensors
//      temp = TheQuadSensors.getQuadSensor(_control[1]-1);
//      TheIndicators.set(i,temp+1);
    }
    else {
      TheIndicators.set(i,0);
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
#if 0
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
#endif
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
  if (ret_val = client.connect(server, 2048)) {
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
  EEPROM.get(ADDRESS_NUMBER_QUADTURNOUT_BOARDS, temp);
  String cmdLabel, command, tempStr;
  
  index1 = commandBuffer.indexOf(" ");
  index2 = commandBuffer.indexOf(" ",index1+1);
  cmdLabel = commandBuffer.substring(index1+1, index2);
  command = commandBuffer.substring(index2+1);
#ifdef TURNOUT_SYSTEM
  if ( commandBuffer.startsWith("TURNOUT") ) {
    for ( i = 0 ; i < 4*temp ; i ++ ) {
      EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
      tempStr = String(tempstr);
      tempStr.trim();
      if ( tempStr == cmdLabel ) {
        if ( command.startsWith("CLOSED") ) {
#ifdef DBGLVL2
          Serial.print("CommandBuffer: setRemote CLOSED ");
          Serial.println(i);
#endif
          TheTurnouts.setRemote(i,2);
        } else if ( command.startsWith("THROWN") ) {
#ifdef DBGLVL2
          Serial.print("CommandBuffer: setRemote THROWN ");
          Serial.println(i);
#endif
          TheTurnouts.setRemote(i,1);
        }          
      }
    }
  } else if ( commandBuffer.startsWith("SENSOR") ) {
#ifdef SIGNAL_SYSTEM
    for ( i = 0 ; i < N_SIGNALS ; i++ ) {
      EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i+17,tempstr);
      tempStr = String(tempstr);
      if ( tempStr == cmdLabel ) {
        if ( command.startsWith("ACTIVE") ) {
          stateRemoteSensor[i] = 1;
        } else if ( command.startsWith("INACTIVE") ) {
          stateRemoteSensor[i] = 0;
        }
      }
    }
#endif
  } else if ( commandBuffer.startsWith("SIGNALHEAD") ) {
#ifdef SIGNAL_SYSTEM
    for ( i = 0 ; i < N_SIGNALS ; i++ ) {
      EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i+10,tempstr);
      tempStr = String(tempstr);
      if ( tempStr == cmdLabel ) {
        stateLeadingSignal[i] = signalAspectToCode(command);
//        Serial.println(stateLeadingSignal[i]);       
      }
    }  
#endif  
  } else if ( commandBuffer.startsWith("NODE jmri") ) {
    jmri_running = 1;
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
#if 0
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
//  544   567   Signal 1/1 [ 0 - sensor ; 1 - turnout1 ; 2 - turnout2 ; [3-9] signalhead name ; 
//                            [10-16] leading (remote signalhead name ; [17-23] remote sensor name ]
//  568   591   Signal 1/2
//  592   615   Signal 1/3
//  616   639   Signal 1/4
//  640   663   Signal 2/1
//  664   687   Signal 2/2
//  688   711   Signal 2/3
//  712   735   Signal 2/4
//  736   759   Signal 3/1
//  760   783   Signal 3/2
//  784   807   Signal 3/3
//  808   831   Signal 3/4
//  832   855   Signal 4/1
//  856   879   Signal 4/2
//  880   903   Signal 4/3
//  904   927   Signal 4/4
  EEPROM.get(20,contents);
  for ( i = 0; i < 4*contents ; i++ ) {
    SignalObject sig;
    EEPROM.get(SIGNAL_BASEADD + SIZE_OF_SIGNAL*i,sig);
    if (1) {
      Serial.print("Signal ");
      Serial.print(int(i/4));
      Serial.print("/");
      Serial.print(i-int(i/4)*4);
      Serial.print(" - sensor ");
      Serial.print(sig.sensorNumber);
      Serial.print(" turnouts ");
      Serial.print(sig.turnout1Number); 
      Serial.print(" ");
      Serial.print(sig.turnout2Number);
      Serial.print(" name ");
      Serial.print(sig.signalHead);
      Serial.print(" leading ");
      Serial.print(sig.leadingSignalHead);
      Serial.print(" remote ");
      Serial.println(sig.remoteSensor);
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
#endif
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
//  544   567   Signal 1/1 [ 0 - sensor ; 1 - turnout1 ; 2 - turnout2 ; [3-9] signalhead name ; 
//                            [10-16] leading (remote signalhead name ; [17-23] remote sensor name ]
//  568   591   Signal 1/2
//  592   615   Signal 1/3
//  616   639   Signal 1/4
//  640   663   Signal 2/1
//  664   687   Signal 2/2
//  688   711   Signal 2/3
//  712   735   Signal 2/4
//  736   759   Signal 3/1
//  760   783   Signal 3/2
//  808   831   Signal 3/4
//  832   855   Signal 4/1
//  856   879   Signal 4/2
//  880   903   Signal 4/3
//  904   927   Signal 4/4
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
