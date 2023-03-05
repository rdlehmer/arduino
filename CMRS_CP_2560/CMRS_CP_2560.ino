                                              // Version v0.4.1
// Ron Lehmer   2023-03-04
//
// For the Arduino Uno R3/Mega 2560
//

#define SIGNAL_SYSTEM
#define TURNOUT_SYSTEM
#define SD_SYSTEM
#define NETWORK_SYSTEM

//
// Library Include Files
//
#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PCF8574.h>
#include <MCP23017.h>
#ifdef SD_SYSTEM
#include <SPI.h>
#include <SD.h>
#endif

//
// Define Device Objects
//

#define PCF8574_LOW_MEMORY

#ifdef TURNOUT_SYSTEM
PCF8574 toggle1(32);
PCF8574 toggle2(33);
PCF8574 indicator1(56);
PCF8574 indicator2(57);
PCF8574 turnout1(58);
MCP23017 turnoutA(36);
MCP23017 turnoutB(37);
#endif

#ifdef SIGNAL_SYSTEM
PCF8574 sensor1(34);
PCF8574 sensor2(35);
MCP23017 signal1(36);
MCP23017 signal2(37);
MCP23017 signal3(38);
MCP23017 signal4(39);
#endif

EthernetClient client;

//
// Global Constants
//
#define N_TOGGLES 4
#define N_TURNOUTS 2
#define N_QUADTURNOUTS 8
#define N_QUADSENSORS 16
#define N_SENSORS 8
#define N_INDICATORS 8
#define N_SIGNALS 16

#define I2C_BASEADD 16
#define I2C_NUMTYPES 6

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

//
// Global Variables
//
int run_mode = 1;
byte mac[6];
byte ipAddr[4];
byte ipServer[4];

byte n_active[I2C_NUMTYPES];

#ifdef TURNOUT_SYSTEM
static int stateToggle[N_TOGGLES];
static int stateTogglePrev[N_TOGGLES];

static byte turnoutToggles[N_TURNOUTS];
static byte stateTurnout[N_TURNOUTS];

static byte turnoutQuadToggles[N_QUADTURNOUTS];
static byte stateQuadTurnout[N_QUADTURNOUTS];

static byte stateQuadSensor[N_QUADSENSORS];
static byte stateQuadSensorPrev[N_QUADSENSORS];

static byte stateIndicators[N_INDICATORS];
#endif

#ifdef SIGNAL_SYSTEM
static byte stateSensor[N_SENSORS];
static byte stateSensorPrev[N_SENSORS];

static byte stateSignal[N_SIGNALS];
static byte stateSignalPrev[N_SIGNALS];
static byte stateLeadingSignal[N_SIGNALS];
static byte stateRemoteSensor[N_SIGNALS];
#endif

static int run_first_time = 1;
static unsigned long prevTime = 0;
static int statusCounts = 0;
String commandBuffer;
String receiveBuffer;
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

static byte flash;
static byte jmri_running;

static byte SERIALON = 1;

///////////////////////////////////////////////////////////////////////////////
//
// EEPROM Initialization
//
///////////////////////////////////////////////////////////////////////////////
void eeprom_init() {
  int i,j;
  byte temp;
  for ( i = 0 ; i < 8 ; i++ ) {
    EEPROM.get(TURNOUT_BASEADD+i*SIZE_OF_TURNOUT,temp);
    if ( temp == 255 ) {
      for ( j = 0 ; j < SIZE_OF_TURNOUT ; j++ ) {
        EEPROM.update(TURNOUT_BASEADD+i*SIZE_OF_TURNOUT+j,byte(0));
      }
    }
  }
  for ( i = 0 ; i < 16; i++ ) {
    EEPROM.get(SENSOR_BASEADD+i*SIZE_OF_SENSOR,temp);
    if ( temp == 255 ) {
      for ( j = 0 ; j < SIZE_OF_SENSOR ; j++ ) {
        EEPROM.update(SENSOR_BASEADD+i*SIZE_OF_SENSOR+j,byte(0));
      }
    }
  }
  for ( i = 0 ; i < 16; i++ ) {
    EEPROM.get(SIGNAL_BASEADD+i*SIZE_OF_SIGNAL,temp);
    if ( temp == 255 ) {
      for ( j = 0 ; j < SIZE_OF_SIGNAL ; j++ ) {
        EEPROM.update(SIGNAL_BASEADD+i*SIZE_OF_SIGNAL+j,byte(0));
      }
    }
  }
  for ( i = 0 ; i < 16; i++ ) {
    EEPROM.get(INDICATOR_BASEADD+i,temp);
    if ( temp == 255 ) {
      EEPROM.update(INDICATOR_BASEADD+i,byte(0));
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
        if ( SERIALON ) {
          Serial.print(baseAdd+i);
          Serial.print(" | ");
        }
      }
      EEPROM.get(baseAdd+i,contents);
      if ( consoleBuffer.startsWith("ex/h") ) {
        if ( SERIALON ) Serial.print(contents,HEX);
      }
      else if ( consoleBuffer.startsWith("ex/s") ) {
        if ( SERIALON ) Serial.print(char(contents));
      }
      else {
        if ( SERIALON ) Serial.print(contents);
      }
      if ( SERIALON ) Serial.print(" ");
      if ( (( i % 8 ) == 7 ) ) {
        if ( SERIALON ) Serial.println();
      }
    }
    if ( SERIALON ) Serial.println();
  }
}

byte hexToByte(char arg_char) {
  if ( (arg_char >= '0') && (arg_char <= '9') )
    return (byte)(arg_char - 48);
  if ( (arg_char >= 'A') && (arg_char <= 'F') )
    return (byte)(arg_char - 55);
  return 0;
}

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
      if ( SERIALON ) { 
        Serial.print("I2C device found at ");
        if ( address < 16 )
          Serial.print("0");
        Serial.print(address);
        Serial.println(" !");
      }
      nDevices++;
    }
    else if (( error == 4 ) && ( SERIALON )) {
      Serial.print("Unknown error at address 0x");
      if ( address < 16 )
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if ( SERIALON ) {
    if ( nDevices == 0 )
      Serial.println("No I2C devices found\n");
    else
      Serial.println("I2C Scan Done\n");
  }
  //delay(500); // remove wait to reduce time for turnouts to move.
}

#ifdef NETWORK_SYSTEM
///////////////////////////////////////////////////////////////////////////////
//
// Internet Services
//
///////////////////////////////////////////////////////////////////////////////
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
    if ( SERIALON ) Serial.println("Ethernet Hardware Error");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    if ( SERIALON ) Serial.println("Ethernet cable is not connected.");
    delay(500);
  }

  // give the Ethernet shield a second to initialize:
  if ( Efirst == 1 ) {
//    delay(1000);
    Efirst = 0;
  }
  if ( SERIALON ) Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (ret_val = client.connect(server, 2048)) {
    if ( SERIALON ) Serial.println("connected");
  } else {
    // if you didn't get a connection to the server:
    if ( SERIALON ) Serial.println("connection failed");
  }
}
#endif

///////////////////////////////////////////////////////////////////////////////
//
// SETUP device
//
///////////////////////////////////////////////////////////////////////////////
void setup() {
//
//  Initialize EEPROM 
//
  eeprom_init();

//
//  Initialize variables
//
  flash = HIGH;
  jmri_running = 0;
  int i;
  byte temp;

//
// Start the Serial interface to the PC via USB
//
  Serial.begin(9600); 
  if ( SERIALON ) Serial.println("CMRS Signal and Turnout 20230304 v0.4.1");

//
// Initialize the W5100 board configuration    
//  
  Ethernet.init(10); // Arduino Ethernet board SS  
#ifdef SD_SYSTEM
  sdCardManager();
#endif
//  
// Start the I2C Bus
//
  Wire.setClock(100000);  // Slow mode 10kHz // Normal mode 100kHz
  Wire.begin();

//
// Scan I2C bus and report devices
//
  scan_i2c();  // v0.3.7 re-enabled

//
// Get board types from EEPROM (addresses 16 - 21)  
//
  EEPROM.get(I2C_BASEADD, n_active);

  if ( ( 2*n_active[4] ) > N_SIGNALS ) {
    n_active[4] = 0;
  }
// Set all pins to high impedance inputs by default
#ifdef TURNOUT_SYSTEM
  if ( n_active[0] > 0 ) {
    for ( i = 0; i < 8; i++ ) {
      toggle1.pinMode(i, INPUT_PULLUP);
      if ( n_active[0] > 1 ) toggle2.pinMode(i, INPUT_PULLUP);
    }
  }
  if ( n_active[2] > 0 ) {
    for ( i = 0; i < 16; i++ ) {
      turnoutA.pinMode(i, INPUT_PULLUP);
      if ( n_active[2] > 1 ) turnoutB.pinMode(i, INPUT_PULLUP);
    }
  }
  if ( n_active[3] > 0 ) {
    for ( i = 0; i < 8; i++ ) {
      indicator1.pinMode(i, OUTPUT, HIGH);
      if ( n_active[3] > 1 ) indicator2.pinMode(i, OUTPUT, HIGH);
    }
  }
  if ( n_active[5] > 0 ) {
    for ( i = 0; i < 8; i++ ) {
      turnout1.pinMode(i, INPUT_PULLUP);
    }    
  }
#endif

#ifdef SIGNAL_SYSTEM
  if ( n_active[1] > 0 ) {
    for ( i = 0; i < 8; i++ ) {
      sensor1.pinMode(i, INPUT_PULLUP);
      if ( n_active[1] > 1 ) sensor2.pinMode(i, INPUT_PULLUP);
    }
  }
  if ( n_active[4] > 0 ) {
    for ( i = 0 ; i < 16 ; i++ ) {
      signal1.pinMode(i, OUTPUT, HIGH);
      if ( n_active[4] > 1 ) signal2.pinMode(i, OUTPUT, HIGH);
      if ( n_active[4] > 2 ) signal3.pinMode(i, OUTPUT, HIGH);
      if ( n_active[4] > 3 ) signal4.pinMode(i, OUTPUT, HIGH);      
    }  
  }
#endif


// Get turnout control data from EEPROM (which toggle controls and 
// last commanded state of turnout)
// Also set pins to outputs
 
  for ( i = 0; i < 4*n_active[2]; i++ ) {
    turnoutQuadToggles[i] = EEPROM.read(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i);  
//    Serial.println("turnoutQuadToggles");
//    Serial.println(turnoutQuadToggles[i]); 
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
      stateQuadTurnout[i] = 1;
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
      stateQuadTurnout[i] = 0;
    }    
  }
    
  for ( i = 0; i < 2*n_active[5]; i++ ) {
    turnoutToggles[i] = EEPROM.read(TURNOUT_BASEADD+SIZE_OF_TURNOUT*i);   
    EEPROM.get(TURNOUT_STATE_BASEADD+i,temp);
    if (temp == 1) {
      turnout1.pinMode(i, OUTPUT, LOW);
      stateTurnout[i] = 1;
    }
    else {
      turnout1.pinMode(i, OUTPUT, HIGH);
      stateTurnout[i] = 0;
    }
  }   

// Start the devices that should be active
#ifdef TURNOUT_SYSTEM
  if ( n_active[0] > 0 ) {
    toggle1.begin();
    if ( n_active[0] > 1 ) toggle2.begin();
  }
#if 0  
  if ( n_active[2] > 0 ) {
    turnoutA.begin();
    if ( n_active[2] > 1 ) turnoutB.begin();
  }
#endif
  if ( n_active[3] > 0 ) {
    indicator1.begin();
    if ( n_active[3] > 1 ) indicator2.begin();
  }
  if ( n_active[5] > 0 ) {
    turnout1.begin();
  }
#endif

#ifdef SIGNAL_SYSTEM
  if ( n_active[1] > 0 ) {
    sensor1.begin();
    if ( n_active[1] > 1 ) sensor2.begin();
  } 
//  signal1.begin();
#endif
}

///////////////////////////////////////////////////////////////////////////////
//
//  loop()
//
///////////////////////////////////////////////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
  if ( run_mode == 1 )
    loop_run();
  else
    loop_setup();
}

  
void loop_setup() {

}

static long reconnect_timer = 0;
void loop_run() {
#ifdef NETWORK_SYSTEM
  if (( run_first_time == 1 ) && ( reconnect_timer == 0 )) {
    connectServer();
    run_first_time = 0;
    reconnect_timer = 60000;
  }
  static int temp = 0;
  if (client.available()) {
    char c = client.read();
    if ( c != 10 )
      receiveBuffer = String(receiveBuffer+String(c));
    if ( c == 10 ) {
      if ( SERIALON ) Serial.print("Receive: ");
      if ( SERIALON ) Serial.println(receiveBuffer);
      commandBuffer = receiveBuffer;
      receiveBuffer = String();
      processCommandBuffer();
    }
  }
#endif

  // as long as there are bytes in the serial queue,
  // read them and send them out the socket if it's open:
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
      else if (client.connected()) {
        client.print(String(consoleBuffer+String(inChar)));
        if ( SERIALON ) Serial.print("CONSOLE SEND: ");
        if ( SERIALON ) Serial.println(consoleBuffer);
      }
      consoleBuffer = String();
    }
  }
#ifdef NETWORK_SYSTEM
  // if the server's disconnected, stop the client:
  if ( (!client.connected() ) && ( run_first_time == 0 ) ) {
    if ( SERIALON ) Serial.println();
    if ( SERIALON ) Serial.println("disconnecting.");
    client.stop();
    // do nothing:
    run_first_time = 2;  //waiting for timer
    jmri_running = 0;
    reconnect_timer = 60000;
//    while (true) {
//      delay(1);
//    }
  }
#endif

#define TIME_STEP 50
  static byte flipflop = 0;
  unsigned long currentTime = millis();
  if ( currentTime < prevTime )
    prevTime = currentTime; // this takes care of rolling over the counter after 50 days
  if ( currentTime > prevTime + TIME_STEP ) {  // do I/O operations every 20ms
    if ( ( reconnect_timer > 0 ) && ( run_first_time == 2 ) ) {
      reconnect_timer = reconnect_timer - TIME_STEP;
    }
    if ( reconnect_timer <= 0 ) {
      reconnect_timer = 0;
      run_first_time = 1;
    }
    if ( flipflop == 0 ) {
#ifdef TURNOUT_SYSTEM
      readToggles();
#endif
#ifdef SIGNAL_SYSTEM
      readSensors();
#endif
      flipflop = 1;
    }
    else {
      setTurnouts();
      writeTurnouts();
      setIndicators();
      writeIndicators();
 #ifdef SIGNAL_SYSTEM
      setSignals();
      writeSignals();
 #endif
      flipflop = 0;
    }
    prevTime += TIME_STEP;
    if ( ( statusCounts % (1000/TIME_STEP) ) == (1000/TIME_STEP - 1) ) {
      if ( flash == HIGH ) {
        flash = LOW;
      } else {
        flash = HIGH;
      }    
    }
    statusCounts++;
  }
  if ( ( statusCounts % (60000/TIME_STEP) ) == (60000/TIME_STEP -1) ) { // send status to network every 60 seconds
    sendRegularStatusMessages();
    statusCounts = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
//
// Command Processing from network message
//
///////////////////////////////////////////////////////////////////////////////
void processCommandBuffer() {
  int index1,index2;
  int i;
  char tempstr[6];
  String cmdLabel,command,tempStr;
  index1 = commandBuffer.indexOf(" ");
  index2 = commandBuffer.indexOf(" ",index1+1);
  cmdLabel = commandBuffer.substring(index1+1, index2);
  command = commandBuffer.substring(index2+1);
  if ( commandBuffer.startsWith("TURNOUT") ) {
    for ( i = 0 ; i < 4*n_active[2] ; i ++ ) {
      EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
      tempStr = String(tempstr);
      if ( tempStr == cmdLabel ) {
        if ( command.startsWith("CLOSED") ) {
          if ( stateQuadTurnout[i] != 0 ) {
            stateQuadTurnout[i] = 0;    // v0.3.5 remove response to remote command
//            sendQuadStatusMessage(i);
            EEPROM.update(QUADTURNOUT_STATE_BASEADD+i,byte(0));
            commandSlavedQuadTurnout(i);
          }
        } else if ( command.startsWith("THROWN") ) {
          if ( stateQuadTurnout[i] != 1 ) {
            stateQuadTurnout[i] = 1;
//            sendQuadStatusMessage(i); // v0.3.5 remove response to remote command
            EEPROM.update(QUADTURNOUT_STATE_BASEADD+i,byte(1));
            commandSlavedQuadTurnout(i);
          }          
        }          
      }
    }
    for ( i = 0 ; i < 2*n_active[5] ; i ++ ) {
      EEPROM.get(TURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
      tempStr = String(tempstr);
      if ( tempStr == cmdLabel ) {
        if ( command.startsWith("CLOSED") ) {
          if ( stateTurnout[i] != 0 ) {
            stateTurnout[i] = 0;
            sendDualStatusMessage(i);
            EEPROM.update(TURNOUT_STATE_BASEADD+i,byte(0));
          }
        } else if ( command.startsWith("THROWN") ) {
          if ( stateTurnout[i] != 1 ) {
            stateTurnout[i] = 1;
            sendDualStatusMessage(i);
            EEPROM.update(TURNOUT_STATE_BASEADD+i,byte(1));
          }          
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
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  see if there is a second turnout we need to throw
//
///////////////////////////////////////////////////////////////////////////////
void commandSlavedQuadTurnout(int arg_index) {
  int j;
  byte toggle,temp;
  EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*arg_index,toggle);
  for ( j = 0 ; j < 4*n_active[2] ; j++ ) {
    EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*j,temp);
    if (( j != arg_index) && ( temp == toggle )) {
      stateQuadTurnout[j] = stateQuadTurnout[arg_index];
      sendQuadStatusMessage(j);
      byte val1;
      EEPROM.get(QUADTURNOUT_STATE_BASEADD+arg_index,val1);
      EEPROM.update(QUADTURNOUT_STATE_BASEADD+j,val1);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
//
//  send a status message on a regular (60 second) heartbeat
//
///////////////////////////////////////////////////////////////////////////////
void sendRegularStatusMessages() {
  byte i;
  for ( i = 0 ; i < 2*n_active[5] ; i++ ) {
    sendDualStatusMessage(i);
  }  
  for ( i = 0 ; i < 4*n_active[2] ; i++ ) {
    sendQuadStatusMessage(i);
  }
#ifdef SIGNAL_SYSTEM
  for ( i = 0 ; i < 4*n_active[4] ; i++ ) {
    sendSignalMessage(i);
  }
#endif
#if 0
  for ( i = 0 ; i < 4*n_active[1] ; i++ ) {
    sendSensorMessage(i);
  }
#endif
}


#ifdef TURNOUT_SYSTEM

///////////////////////////////////////////////////////////////////////////////
//
//  send a turnout message from a Dual turnout module
//
///////////////////////////////////////////////////////////////////////////////
void sendDualStatusMessage(byte i) {
#ifdef NETWORK_SYSTEM
  char tempstr[7];
  if ( ( client.connected() ) && ( jmri_running == 1 ) ) {
    String tempStr;
    EEPROM.get(TURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
    byte temp;
    temp = stateTurnout[i];
    tempStr = String(tempstr);
    if ( ( tempStr.length() != 0 ) && ( turnoutToggles[i] != 0 ) 
        && ( turnoutToggles[i] != 255 ) ) {
      tempStr = String("TURNOUT "+String(tempstr));
      if ( temp == 0 ) {
        tempStr = String(tempStr+" CLOSED");
      }
      else if ( temp == 1 ) {
        tempStr = String(tempStr+" THROWN");       
      }
      else {
        tempStr = String(tempStr+" UNKNOWN");       
      }
      if ( SERIALON ) Serial.print("Send: ");
      if ( SERIALON ) Serial.println(tempStr);
      if (client.connected()) client.println(tempStr);
    }
  }
#endif
}

///////////////////////////////////////////////////////////////////////////////
//
//  send turnout messages from Quad turnout module to JMRI
//
///////////////////////////////////////////////////////////////////////////////
void sendQuadStatusMessage(byte i) {
#ifdef NETWORK_SYSTEM
  char tempstr[7];
  if ( ( client.connected() ) && ( jmri_running == 1 ) ) {
    String tempStr;
    EEPROM.get(QUADTURNOUT_BASEADD+SIZE_OF_TURNOUT*i+1,tempstr);
    byte temp;
    temp = stateQuadTurnout[i];
    tempStr = String(tempstr);
    if ( ( tempStr.length() != 0 ) && ( turnoutQuadToggles[i] != 0 ) 
        && (turnoutQuadToggles[i] != 255 ) ) {
      tempStr = String("TURNOUT "+String(tempstr));
      if ( temp == 0 ) {
        tempStr = String(tempStr+" CLOSED");
      }
      else if ( temp == 1 ) {
        tempStr = String(tempStr+" THROWN");       
      }
      else {
        tempStr = String(tempStr+" UNKNOWN");       
      }
      if ( SERIALON ) Serial.print("Send: ");
      if ( SERIALON ) Serial.println(tempStr);
      if (client.connected()) client.println(tempStr);
    }
  }
#if 0
  Serial.print("Send - ");
  Serial.println(i);
#endif
#endif
}


///////////////////////////////////////////////////////////////////////////////
//
//  Handle toggle switches
//
///////////////////////////////////////////////////////////////////////////////
void readToggles() {
  int i;
  for ( i = 0 ; i < 4*n_active[0] ; i++ ) {
    byte tempa,tempb;
    if ( i < 4 ) {
      tempa = toggle1.digitalRead(2*i);
      tempb = toggle1.digitalRead(2*i+1);
    } else if ( i < 8 ) {
      tempa = toggle2.digitalRead(2*(i-4));
      tempb = toggle2.digitalRead(2*(i-4)+1);
    }
    if ( tempa == LOW ) {
      stateToggle[i] = 1;
    }
    else if ( tempb == LOW ) {
      stateToggle[i] = -1;
    }
    else {
      stateToggle[i] = 0;
    }
#if 0
    if ( stateTogglePrev[i] != stateToggle[i] ) {
      Serial.print(i);
      Serial.print(" ");
      Serial.println(stateToggle[i]);
    }
#endif
    stateTogglePrev[i] = stateToggle[i];
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  Set Turnout states based on Toggles or JMRI commands
//
///////////////////////////////////////////////////////////////////////////////
void setTurnouts() {
  readQuadSensors();
  setDualTurnouts();
  setQuadTurnouts();
}

void setDualTurnouts() {
  int i;
  for ( i = 0; i < 2*n_active[5] ; i++ ) {
    if (( turnoutToggles[i] != 0 ) && ( turnoutToggles[i] != 255 )) {
      if (( stateToggle[turnoutToggles[i]-1] == 1 ) 
           && ( stateTurnout[i] != 1 )) {
        stateTurnout[i] = 1;
        sendDualStatusMessage(i);
        EEPROM.update(TURNOUT_STATE_BASEADD+i,byte(1));
      }
      if (( stateToggle[turnoutToggles[i]-1] == -1 ) 
           && ( stateTurnout[i] != 0 )) {
        stateTurnout[i] = 0;
        sendDualStatusMessage(i);
        EEPROM.update(TURNOUT_STATE_BASEADD+i,byte(0));
      }
    }
  }
}

void setQuadTurnouts() {
  int i,j;
  for ( i = 0; i < 4*n_active[2] ; i++ ) {
    if (( turnoutQuadToggles[i] != 0 ) && ( turnoutQuadToggles[i] != 255 )) {
      if (( stateToggle[turnoutQuadToggles[i]-1] == 1 ) 
           && ( stateQuadTurnout[i] != 1 )) {
        stateQuadTurnout[i] = 1;
        sendQuadStatusMessage(i);
        EEPROM.update(QUADTURNOUT_STATE_BASEADD+i,byte(1));
      }
      if (( stateToggle[turnoutQuadToggles[i]-1] == -1 ) 
           && ( stateQuadTurnout[i] != 0 )) {
        stateQuadTurnout[i] = 0;
        sendQuadStatusMessage(i);
        EEPROM.update(QUADTURNOUT_STATE_BASEADD+i,byte(0));
      }
    } 
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  Write Turnout states to hardware
//
///////////////////////////////////////////////////////////////////////////////
void writeTurnouts() {
  writeDualTurnouts();
  writeQuadTurnouts();
}

void writeDualTurnouts() {
  int i;
  byte tempNumber;
  EEPROM.get(21,tempNumber);
  for ( i = 0 ; i < 2*tempNumber ; i++ ) {
    if (stateTurnout[i] == 1 ) {
      if ( i < 2 ) {
        turnout1.digitalWrite(i, LOW);
      }
    }
    else {
      if ( i < 2 ) {
        turnout1.digitalWrite(i, HIGH);
      }
    }
  }
}

void writeQuadTurnouts() {
  int i;
  byte tempNumber;
  EEPROM.get(18,tempNumber);
  for ( i = 0 ; i < 4*tempNumber ; i++ ) {
    if (stateQuadTurnout[i] == 0 ) {
      if ( i < 4 ) {
        turnoutA.digitalWrite(2*i, LOW);
        turnoutA.digitalWrite(2*i+1, HIGH);
      }
      else if ( i < 8 ) {
        turnoutB.digitalWrite(2*(i-4), LOW);
        turnoutB.digitalWrite(2*(i-4)+1, HIGH);
      }
    }
    else {
      if ( i < 4 ) {
        turnoutA.digitalWrite(2*i, HIGH);
        turnoutA.digitalWrite(2*i+1, LOW);
      }
      else if ( i < 8 ) {
        turnoutB.digitalWrite(2*(i-4), HIGH);
        turnoutB.digitalWrite(2*(i-4)+1, LOW);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  get Sensors from Quad Turnouts
//
///////////////////////////////////////////////////////////////////////////////
void readQuadSensors() {
  QuadSensorObject quadSensor;
  int i;
  for ( i = 0 ; i < N_QUADSENSORS ; i++ ) {
    EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*i, quadSensor);
    if ( (quadSensor.boardNumber != 0) && (quadSensor.boardNumber <= n_active[2]) 
        && (quadSensor.sensorChannel != 0) && (quadSensor.sensorChannel <= 8) 
        && (quadSensor.sensorNumber != 0) && (quadSensor.sensorNumber != 255)) {
      byte temp = 0;
      switch ( quadSensor.boardNumber ) {
        case 1: temp = turnoutA.digitalRead(quadSensor.sensorChannel+7);
              break;
        case 2: temp = turnoutB.digitalRead(quadSensor.sensorChannel+7);
              break;
// sensorChannel definition changed for Rev C and beyond boards
      }
      if ( temp == LOW ) {
        stateQuadSensor[i] = 1;
      } else {
        stateQuadSensor[i] = 0;
      }
      if ( stateQuadSensor[i] != stateQuadSensorPrev[i] ) {
        sendQuadSensorMessage(i);
      }
      stateQuadSensorPrev[i] = stateQuadSensor[i];
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  set indicator state
//
///////////////////////////////////////////////////////////////////////////////
void setIndicators() {
  int i;
  for ( i = 0; i < 4*n_active[3]; i++ ) {
    byte temp[2];
    EEPROM.get(INDICATOR_BASEADD+2*i,temp);
    if ( temp[0] != 0 ) {
      stateIndicators[i] = stateQuadTurnout[temp[0]-1];
    }
    else if ( temp[1] != 0 ) {
      stateIndicators[i] = stateQuadSensor[temp[1]-1];
    }
    else {
      stateIndicators[i] = 2;
    }
  }
/*
  Serial.print("Turnout:  ");
  for ( i = 0 ; i < 8 ; i++ ) {
    Serial.print(stateQuadTurnout[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Sensor:   ");
  for ( i = 0 ; i < 8 ; i++ ) {
    Serial.print(stateQuadSensor[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Indicator: ");
  for ( i = 0 ; i < 8 ; i++ ) {
    Serial.print(stateIndicators[i]);
    Serial.print(" ");
  }
  Serial.println();
*/  
}

    

///////////////////////////////////////////////////////////////////////////////
//
//  write indicator state
//
///////////////////////////////////////////////////////////////////////////////
void writeIndicators() {
  int i;
  for ( i = 0; i < 4*n_active[3]; i++ ) {
    if ( i < 4 ) {
      switch ( stateIndicators[i] ) {
        case 0 : indicator1.digitalWrite(2*i, LOW);
                 indicator1.digitalWrite(2*i+1, HIGH);
                 break;
        case 1 : indicator1.digitalWrite(2*i, HIGH);
                 indicator1.digitalWrite(2*i+1, LOW);
                 break;
        default : indicator1.digitalWrite(2*i, HIGH);
                 indicator1.digitalWrite(2*i+1, HIGH);
                 break; 
      }
    }
    else {
      switch ( stateIndicators[i] ) {
        case 0 : indicator2.digitalWrite(2*(i-4), LOW);
                 indicator2.digitalWrite(2*(i-4)+1, HIGH);
                 break;
        case 1 : indicator2.digitalWrite(2*(i-4), HIGH);
                 indicator2.digitalWrite(2*(i-4)+1, LOW);
                 break;
        default : indicator2.digitalWrite(2*(i-4), HIGH);
                 indicator2.digitalWrite(2*(i-4)+1, HIGH);
                 break; 
      }      
    }
  }
}



///////////////////////////////////////////////////////////////////////////////
//
//  send a track sensor message to JMRI
//
///////////////////////////////////////////////////////////////////////////////
void sendQuadSensorMessage(byte i) {
#ifdef NETWORK_SYSTEM
  char tempstr[7];
  if ( ( client.connected() ) && ( jmri_running == 1 ) ) {
    String tempStr;
    EEPROM.get(QUADTURNOUT_SENSOR_BASEADD+SIZE_OF_QUADTURNOUT_SENSOR*i+2,tempstr);
    tempStr = String(tempstr);
    if ( tempStr.length() != 0 ) {
      tempStr = String("SENSOR "+tempStr);
      if ( stateQuadSensor[i] == 0 ) {
        tempStr = String(tempStr+" INACTIVE");
      }
      else if (stateQuadSensor[i] == 1 ) {
        tempStr = String(tempStr+" ACTIVE");
      }
      else {
        tempStr = String(tempStr+" UNKNOWN");
      }
      if ( SERIALON ) Serial.print("Send: ");
      if ( SERIALON ) Serial.println(tempStr);
      if (client.connected()) client.println(tempStr);
    }
  }
#endif
}


#endif

#ifdef SIGNAL_SYSTEM

///////////////////////////////////////////////////////////////////////////////
//
//  send a track sensor message to JMRI
//
///////////////////////////////////////////////////////////////////////////////
void sendSensorMessage(byte i) {
#ifdef NETWORK_SYSTEM
  char tempstr[7];
  if ( ( client.connected() ) && ( jmri_running == 1 ) ) {
    String tempStr;
    EEPROM.get(SENSOR_BASEADD+SIZE_OF_SENSOR*i+1,tempstr);
    tempStr = String(tempstr);
    if ( tempStr.length() != 0 ) {
      tempStr = String("SENSOR "+tempStr);
      if ( stateSensor[i] == 0 ) {
        tempStr = String(tempStr+" INACTIVE");
      }
      else if (stateSensor[i] == 1 ) {
        tempStr = String(tempStr+" ACTIVE");
      }
      else {
        tempStr = String(tempStr+" UNKNOWN");
      }
      if ( SERIALON ) Serial.print("Send: ");
      if ( SERIALON ) Serial.println(tempStr);
      if (client.connected()) client.println(tempStr);
    }
  }
#endif
}


///////////////////////////////////////////////////////////////////////////////
//
//  Handle track sensors
//
///////////////////////////////////////////////////////////////////////////////
static byte sensorDebounce[N_SENSORS];
static byte stateTemp[N_SENSORS];
void readSensors() {
  int i;
  int temp;
  for ( i = 0 ; i < 4*n_active[1] ; i++ ) {
    if ( i < 4 ) {
      temp = sensor1.digitalRead(i);
//      Serial.print(temp);
    }
    else if ( i < 8 ) {
      temp = sensor2.digitalRead(i-4);
    }
    if ( temp == LOW ) {
      stateSensor[i] = 1;
    }
    else {
      stateSensor[i] = 0;
    }
    if ( stateSensor[i] != stateSensorPrev[i] ) {
      sendSensorMessage(i);
    }
    stateSensorPrev[i] = stateSensor[i];
  }
//  Serial.println();
}

///////////////////////////////////////////////////////////////////////////////
//
//  Set Signal states based on inputs from sensors, turnouts, and JMRI
//
///////////////////////////////////////////////////////////////////////////////
void setSignals() {
  SignalObject Signal;
  int i;
  byte redSignal;
  redSignal = 0;
  for ( i = 0 ; i < 4*n_active[4] ; i++ ) {
    EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i,Signal);
    if ( (String(Signal.signalHead)).length() != 0 ) {
      if ( Signal.turnout1Number > 0 ) {
        if ( stateTurnout[Signal.turnout1Number-1] == 1 ) {
          redSignal = 1;
//          if ( i == 0 ) Serial.print("A");
        }
      }
      if ( Signal.turnout2Number > 0 ) {
        if ( stateTurnout[Signal.turnout2Number-1] == 1 ) {
          redSignal = 1;
//          if ( i == 0 ) Serial.print("B");
        }
      }
      if ( Signal.sensorNumber > 0 ) {
        if ( stateSensor[Signal.sensorNumber-1] == 1 ) {
          redSignal = 1;
//          if ( i == 0 ) Serial.print("C");
        }
      }
      if ( (String(Signal.remoteSensor)).length() != 0 ) {
        if ( stateRemoteSensor[i] == 1 ) {
          redSignal = 1;
//          if ( i == 0 ) Serial.print("D");
        }
      }
      stateSignal[i] = 3; // Green
      if ( redSignal == 1 ) {
        stateSignal[i] = 1; // Red
      } else {            // DRK, RED, YLW, GRN, LUN, F RD, F YL, F GR
        byte mapToLead[8] = { 3,   2,   3,   3,   2,   2,    3  ,  3 };
        stateSignal[i] = mapToLead[stateLeadingSignal[i]];
      }
//      if ( i == 0 ) Serial.println(stateSignal[i]);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//  Write Signal states to hardware
//
///////////////////////////////////////////////////////////////////////////////
static byte signal_first_time = 1;
void writeSignals() {
  int i;
  for ( i = 0 ; i < 4*n_active[4] ; i++ ) {
    switch ( stateSignal[i] ) {
      case 0 : signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 1 : signal1.digitalWrite(3*i, LOW);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 2 : signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, LOW);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 3 : signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, LOW);
               break;
      case 4 : signal1.digitalWrite(3*i, HIGH);  // LUNAR not implemented yet
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 5 : signal1.digitalWrite(3*i, flash);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 6 : signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, flash);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
      case 7 : signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, flash);
               break;
      default: signal1.digitalWrite(3*i, HIGH);
               signal1.digitalWrite(3*i+1, HIGH);
               signal1.digitalWrite(3*i+2, HIGH);
               break;
    }
    if ( ( stateSignal[i] != stateSignalPrev[i] ) && ( signal_first_time == 0 ) ) {
      sendSignalMessage(i);
    }
    stateSignalPrev[i] = stateSignal[i];
    signal_first_time = 0;
  }
}




///////////////////////////////////////////////////////////////////////////////
//
// Return index from signal aspect name
//
///////////////////////////////////////////////////////////////////////////////
byte signalAspectToCode(String arg_string) {
  byte ret_val;
  if ( arg_string == "DARK") {
    ret_val = 0;
  } else if ( arg_string == "RED" ) {
    ret_val = 1;
  } else if ( arg_string == "YELLOW" ) {
    ret_val = 2;
  } else if ( arg_string == "GREEN" ) {
    ret_val = 3;
  } else if ( arg_string == "LUNAR" ) {
    ret_val = 4;
  } else if ( arg_string == "FLASHRED" ) {
    ret_val = 5;
  } else if ( arg_string == "FLASHYELLOW" ) {
    ret_val = 6;
  } else if ( arg_string == "FLASHGREEN" ) {
    ret_val = 7;
  } 
  return(ret_val);
}

///////////////////////////////////////////////////////////////////////////////
//
// Return send signal message 
//
///////////////////////////////////////////////////////////////////////////////
void sendSignalMessage(byte i) {
#ifdef NETWORK_SYSTEM
  char tempstr[7];
  if ( ( client.connected() ) && ( jmri_running == 1 ) ) {
    String tempStr;
    EEPROM.get(SIGNAL_BASEADD+SIZE_OF_SIGNAL*i+3,tempstr);
    tempStr = String(tempstr);
    if ( tempStr.length() != 0 ) {
      tempStr = String("SIGNALHEAD "+tempStr);
      switch ( stateSignal[i] ){
        case 0 : tempStr = String(tempStr + " DARK");
                 break;
        case 1 : tempStr = String(tempStr + " RED");
                 break;
        case 2 : tempStr = String(tempStr + " YELLOW");
                 break;
        case 3 : tempStr = String(tempStr + " GREEN");
                 break;
        case 4 : tempStr = String(tempStr + " LUNAR");
                 break;
        case 5 : tempStr = String(tempStr + " FLASHRED");
                 break;
        case 6 : tempStr = String(tempStr + " FLASHYELLOW");
                 break;
        case 7 : tempStr = String(tempStr + " FLASHGREEN");
                 break;
        default: tempStr = String(tempStr + " UNKNOWN");
                 break;                 
      }
      if ( SERIALON ) Serial.print("Send: ");
      if ( SERIALON ) Serial.println(tempStr);
      if (client.connected()) client.println(tempStr);
    }
  }
#endif
}

#endif

/*
void setMacAddress(String arg_str) {
  byte mactemp[6];
  char tempchar[3];
  String tempstr = arg_str.substring(8);
  for (byte octet = 0; octet < 6; octet++) {
    if ( octet < 5 ) {
      int pos = tempstr.indexOf("-");
      if ( pos > 0) {
        tempstr.getBytes(tempchar,3);
        Serial.println(tempchar);
        mactemp[octet] = hexToByte(tempchar[0])*16 + hexToByte(tempchar[1]);
        tempstr = tempstr.substring(pos+1);
      }
    } else {
      tempstr.getBytes(tempchar,3);
      Serial.println(tempchar);
      mactemp[octet] = hexToByte(tempchar[0])*16 + hexToByte(tempchar[1]);
    }
  }
  EEPROM.put(0,mactemp);
}
*/

#ifdef SD_SYSTEM

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
      for ( i = 0; i < 1024; i++ ) {
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
    else if ( SERIALON ) {
      Serial.print("Error: unable to open file ");
      Serial.print(filename.c_str());
      Serial.println(" for write.");
    }
  }
  else {
    if ( SERIALON ) Serial.println("Error: no filename");
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
            if ( SERIALON ) Serial.print("#");
          }
          dataBuffer = String();
          i++;
        }
      }
// more to do here
      myFile.close();
      if ( SERIALON ) Serial.println();
    }
    else {
      if ( SERIALON ) Serial.println("Error: Unable to open file for read.");
    }
  }
  else {
    if ( SERIALON ) Serial.println("Error: no filename");
  }
}

void sdCardManager() {
  byte sdSystemDone = 0;
  if ( !SD.begin(4) ) {
    if ( SERIALON ) Serial.println("No SD card present. Starting...");
    return;
  }
  else {
    if ( SERIALON ) Serial.println("SD Card present. Starting console...(quit) to end.");
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


#endif


void show_configuration() {
  int i;
  byte contents;
  if ( SERIALON ) {
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
//  006   007   undefined
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
//  022   039   undefined
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
  for ( i = 0; i < 4*n_active[2] ; i++ ) {
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
  for ( i = 0 ; i < 4*n_active[2] ; i++ ) {
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
  for ( i = 0; i < 2*n_active[5] ; i++ ) {
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
  for ( i = 0; i < 4*n_active[1] ; i++ ) {
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
  for ( i = 0; i < 4*n_active[4] ; i++ ) {
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
  for ( i = 0 ; i < 4*n_active[3] ; i++ ) {
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
  }
// 
}

//
// EEPROM memory map
//
// start  end     Function
//  000   005   MAC Address (hex)
//  006   007   undefined
//  008   011   IP address of Controller
//  012   015   IP address of JMRI SimpleServer (running port 2048)
//  016         Number of Toggle Boards starting with add 32 (32 - 33)
//  017         Number of Sensor Boards starting with add 34 (34 - 35)
//  018         Number of Quad Turnout Boaards starting with add 36 (36 - 39)
//  019         Number of Indicator Boards
//  020         Number of Signal Boards
//  021         Number of Dual Turnout Proto Boards starting with add 56
//  022   039   undefined
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
// 
