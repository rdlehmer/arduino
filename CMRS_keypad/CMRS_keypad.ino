                                             // Version v0.6.3a
// Ron Lehmer   2023-12-21
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

#define SD_SYSTEM
//#define NETWORK_SYSTEM
//#define POWER_SYSTEM
//#define TURNOUT_SYSTEM
#define KEYPAD_SYSTEM

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
//#include <MCP23017.h>
#include <LiquidCrystal_I2C.h>

///
/// Global variables
///

PCF8574 PanelKeypad(0x026);
LiquidCrystal_I2C PanelLCDisplay(0x027, 16, 2);

// Define the keypad pins
const byte ROWS = 4; 
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','4','7','*'},
  {'2','5','8','0'},
  {'3','6','9','#'},
  {'A','B','C','D'}
};

byte MAXYARDTRACKS = 16;

char scratchpad[3] = "00\0";
int col_count = -1;
int open_track = 0;
unsigned long interlockTime;
int interlockStatus = 0;

String consoleBuffer;
String commandBuffer;

struct StringObject {
  char nameLabel[13];   // specifically for the KEYPAD system
};

struct TrackMapObject {
  byte turnout[16];
};

static unsigned long prevTime = 0;
static long counts = 0;
static unsigned long tempTime = 0;
static int run_first_time = 1;

String serial1ReceiveBuffer;
String serial2ReceiveBuffer;


///
///  BEGIN SETUP
///

void setup() {
  Serial.begin(9600);
  eeprom_init(); 
  Serial.println("CMRS CP_2560_keypad v0.6.3a 2023-12-21");
#ifdef SD_SYSTEM
  Serial.println("Starting SD System...");
//  Ethernet.init(10); // Arduino Ethernet board SS  
  sdCardManager();
#endif
  Serial.println("Starting I2C System...");
  Wire.setClock(100000);  // Slow mode 10kHz // Normal mode 100kHz
  Wire.begin();
  scan_i2c();

  EEPROM.get(976,MAXYARDTRACKS);
  Serial1.begin(9600);
  Serial2.begin(9600);
  keypad_init();
  PanelKeypad.begin();
  PanelLCDisplay.begin(40,2);
  LCDisplay_init();

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

  char keystroke;
  keystroke = read_keypad();
  if ( keystroke != NULL ) {
    if (( keystroke == '*' ) || ( keystroke == 'D' )) {
      line_for_mainline();
    }
    else if (( keystroke == '#' ) || ( keystroke == 'A' )) {
      line_for_yard(-1);
    }
    else if ( keystroke == 'B' ) {
      track_power_on(-1);
    }
    else if ( keystroke == 'C' ) {
      track_power_off(-1);
    }
    else {
      update_scratchpad(keystroke);
    }
  }
  scan_column();

  if (Serial1.available()) {
    int c = Serial1.read();
    if ( c != 10 )
      serial1ReceiveBuffer = String(serial1ReceiveBuffer+String(c));
    if ( c == 10 ) {
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
  if (Serial2.available()) {
    int c = Serial2.read();
    if ( c != 10 )
      serial2ReceiveBuffer = String(serial2ReceiveBuffer+String(c));
    if ( c == 10 ) {
      int j = serial2ReceiveBuffer.length();
      for ( int k = 0 ; k < j/2 ; k++ ) {
        char t1 = (char)atoi((serial2ReceiveBuffer.substring(2*k,2*k+2)).c_str());
        commandBuffer = String(commandBuffer+String(t1));
      }
      Serial.println(commandBuffer);
      serial2ReceiveBuffer = String();
      processCommandBuffer();  // This is where we process incoming messages
    }
  }

  if ( currentTime > ( prevTime + TIME_STEP ) ) {
    prevTime += TIME_STEP;
  }
}


void keypad_init() {
  PanelKeypad.pinMode(P0, INPUT_PULLUP);
  PanelKeypad.pinMode(P1, INPUT_PULLUP);
  PanelKeypad.pinMode(P2, INPUT_PULLUP);
  PanelKeypad.pinMode(P3, INPUT_PULLUP);
  PanelKeypad.pinMode(P4, OUTPUT, HIGH);
  PanelKeypad.pinMode(P5, OUTPUT, HIGH);
  PanelKeypad.pinMode(P6, OUTPUT, HIGH);
  PanelKeypad.pinMode(P7, OUTPUT, HIGH);
}

void LCDisplay_init() {
  char temp[13];
  EEPROM.get(977,temp);
  PanelLCDisplay.clear();
  PanelLCDisplay.backlight();
  PanelLCDisplay.setCursor(0,0);
  PanelLCDisplay.printstr(temp);  
  update_scratchpad('0');
  LCDisplayShowPowerLines();
}

void recover() {
  byte temp;
  EEPROM.get(959,temp);
#ifdef DBGLVL1
  Serial.print("\nEEPROM returned ");
  Serial.println(temp, DEC);
#endif
  if ( temp == 0 ) {
    line_for_mainline();
  }
  else if ( temp <= MAXYARDTRACKS ) {
    line_for_yard(temp);
  }
  else {
    EEPROM.put(959,0);
  }
}

void scan_column() {
  col_count++;
  if ( col_count > 3 ) col_count = 0;
  if ( col_count == 0 ) {
    PanelKeypad.digitalWrite(P7, HIGH);
    PanelKeypad.digitalWrite(P6, HIGH);
    PanelKeypad.digitalWrite(P5, HIGH);
    PanelKeypad.digitalWrite(P4, LOW);
  }
  else if ( col_count == 1 ) {
    PanelKeypad.digitalWrite(P7, HIGH);
    PanelKeypad.digitalWrite(P6, HIGH);
    PanelKeypad.digitalWrite(P5, LOW);
    PanelKeypad.digitalWrite(P4, HIGH);
  }
  else if ( col_count == 2 ) {
    PanelKeypad.digitalWrite(P7, HIGH);
    PanelKeypad.digitalWrite(P6, LOW);
    PanelKeypad.digitalWrite(P5, HIGH);
    PanelKeypad.digitalWrite(P4, HIGH); 
  }
  else if ( col_count == 3 ) {
    PanelKeypad.digitalWrite(P7, LOW);
    PanelKeypad.digitalWrite(P6, HIGH);
    PanelKeypad.digitalWrite(P5, HIGH);
    PanelKeypad.digitalWrite(P4, HIGH);
  }
}

char read_keypad() {
  uint8_t row1, row2, row3, row4, row;
  static char last_char = NULL;
  char ret_val;
  static int counter = 0;
  row1 = PanelKeypad.digitalRead(P0);
  row2 = PanelKeypad.digitalRead(P1);
  row3 = PanelKeypad.digitalRead(P2);
  row4 = PanelKeypad.digitalRead(P3);
  if ( ( row1 == HIGH ) && ( row2 == HIGH ) && ( row3 == HIGH ) && ( row4 == HIGH ) ) {
    ret_val = NULL;
    counter++;
    if ( counter > 200 ) {
      last_char = NULL;
      counter = 0;
    }
  }
  else {
    counter = 0;
    if ( row1 == LOW ) row = 0;
    else if ( row2 == LOW ) row = 1;
    else if ( row3 == LOW ) row = 2;
    else if ( row4 == LOW ) row = 3;
    ret_val = keys[row][col_count];
    if ( ret_val != last_char ) {
      last_char = ret_val;
#ifdef DBGLVL2
      Serial.print(last_char);
#endif
    }
    else {
      ret_val = NULL;
    }
  }
  return ret_val;
}

void update_scratchpad(char arg_char) {
  scratchpad[0] = scratchpad[1];
  scratchpad[1] = arg_char;
  PanelLCDisplay.setCursor(14,0);
  PanelLCDisplay.printstr(scratchpad);
}

void line_for_mainline() {
  char command[20] = "KBTRACK 0 SELECT\n";
  EEPROM.put(959,0);
  Serial.println("\nLine for Main.");
  LCDisplayClearSecondLine();
  LCDisplayShowTrack();
  update_scratchpad('0');
  update_scratchpad('0');
#ifdef DBGLVL1
  Serial.print("Command: ");
  Serial.println(command);
#endif
  Serial1.write(command,strlen(command));
}

void line_for_yard(int arg_val) {
  int temp;
  char command[20] = "KBTRACK ";
  char ctemp[3];
  if ( arg_val != -1 ) {
    temp = arg_val;
  }
  else {
    temp = atoi(scratchpad);
  }
  if ( ( temp > MAXYARDTRACKS ) || ( temp == 0 ) ) {
#ifdef DBGLVL1
    Serial.print("\nError selecting track ");
    Serial.println(scratchpad);
#endif
    update_scratchpad('0');
    update_scratchpad('0');          
  }
  else {
    EEPROM.put(959,byte(temp));   
    LCDisplayClearSecondLine();
    LCDisplayShowTrack();
    update_scratchpad('0');
    update_scratchpad('0');  
    strcat(command,itoa(temp,ctemp,10));
    strcat(command, " SELECT\n");
#ifdef DBGLVL1
    Serial.print("Command: ");
    Serial.println(command);
#endif
    Serial1.write(command,strlen(command));    
  }
}

void track_power_on(int arg_val) {
  int temp;
  char command[20] = "KBPOWER ";
  char ctemp[3];
  if ( arg_val != -1 ) {
    temp = arg_val;
  }
  else {
    temp = atoi(scratchpad);
  }
  if ( ( temp > MAXYARDTRACKS ) || ( temp == 0 ) ) {
#ifdef DBGLVL1
    Serial.print("\nError selecting track ");
#endif
    update_scratchpad('0');
    update_scratchpad('0');          
  }
  else {
    LCDisplayClearSecondLine();
    update_scratchpad('0');
    update_scratchpad('0');  
    EEPROM.put(960+temp-1,byte(1)); 
    strcat(command,itoa(temp,ctemp,10));
    strcat(command," ON\n");
#ifdef DBGLVL1
    Serial.print("Command: ");
    Serial.println(command);
#endif
    Serial1.write(command,strlen(command));
    Serial2.write(command,strlen(command));
    LCDisplayShowPowerLines();
  }
}

void track_power_off(int arg_val) {
  int temp;
  char command[20] = "KBPOWER ";
  char ctemp[3];
  if ( arg_val != -1 ) {
    temp = arg_val;
  }
  else {
    temp = atoi(scratchpad);
  }
  if ( ( temp > MAXYARDTRACKS ) || ( temp == 0 ) ) {
#ifdef DBGLVL1
    Serial.print("\nError selecting track ");
#endif
    Serial.println(scratchpad);
    update_scratchpad('0');
    update_scratchpad('0');          
  }
  else {
    Serial.println(temp, DEC);
    update_scratchpad('0');
    update_scratchpad('0');  
    EEPROM.put(960+temp-1,byte(0)); 
    strcat(command,itoa(temp,ctemp,10));
    strcat(command," OFF\n");
#ifdef DBGLVL1
    Serial.print("Command: ");
    Serial.println(command);
#endif
    Serial1.write(command,strlen(command));
    Serial2.write(command,strlen(command));
    LCDisplayShowPowerLines();
  }
}

void LCDisplayShowTrack() {
  byte temp;
  EEPROM.get(959,temp);
  if ( temp == 0 ) {
    PanelLCDisplay.setCursor(0,1);
    PanelLCDisplay.printstr("Lined for Main\0)");   
  }
  else {
    PanelLCDisplay.setCursor(0,1);
    PanelLCDisplay.printstr("Track \0");
    PanelLCDisplay.printstr(itoa(temp,scratchpad,10)); 
  }
}

void LCDisplayShowPowerLines() {
  int i;
  byte temp;
  char stemp[3];
  String s_display = "";
  for ( i = 0 ; i < 20 ; i++ ) {
    PanelLCDisplay.setCursor(i+20,0);
    PanelLCDisplay.write(' ');
  }  
  for ( i = 0 ; i < 20 ; i++ ) {
    PanelLCDisplay.setCursor(i+20,1);
    PanelLCDisplay.write(' ');
  }
  PanelLCDisplay.setCursor(20,0);
  PanelLCDisplay.printstr("Power");
  for ( i = 0; i < MAXYARDTRACKS ; i++ ) {
    EEPROM.get(960+i,temp);
    if ( temp == 1 ) {
      if ( i > 9 ) {
        PanelLCDisplay.setCursor(31+2*(i-10),1);
      }
      else {
        PanelLCDisplay.setCursor(20+i,1);
      }
      PanelLCDisplay.printstr(itoa(i+1,stemp,10));
    }
  }
  LCDisplayShowTrack();
}

void LCDisplayClearSecondLine() {
  for (int i = 0; i < 20; i++) {
    PanelLCDisplay.setCursor(i,1);
    PanelLCDisplay.write(' ');
  }
}


void processCommandBuffer() {
  int index1,index2;
  int i;
  char tempstr[7];
  String cmdLabel, command, tempStr;
  
  index1 = commandBuffer.indexOf(" ");
  index2 = commandBuffer.indexOf(" ",index1+1);
  cmdLabel = commandBuffer.substring(index1+1, index2);
  command = commandBuffer.substring(index2+1);

  if ( commandBuffer.startsWith("KBPOWER") ) {    // KBPOWER nn ON/OFF via Serial
    byte track1 = byte(cmdLabel.toInt());
    if ( command.startsWith("ON") ) {
      EEPROM.put(960+track1-1,byte(1));
    } else if ( command.startsWith("OFF") ) {
      EEPROM.put(960+track1-1,byte(0));
    }
  } else if ( commandBuffer.startsWith("KBTRACK") ) {  // KBTRACK nn SELECT via Serial
    byte track1 = byte(cmdLabel.toInt());
    EEPROM.put(959,byte(track1));
  }
  commandBuffer = "";
}

///////////////////////////////////////////////////////////////////////////////
//
// EEPROM Initialization
//
///////////////////////////////////////////////////////////////////////////////

void eeprom_init() {
  int i;
  byte temp;
#if 0  
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
#endif
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
#if 0
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
#endif
//  944   959   RESERVED
//  960   975   Track Power Saved State
//  976   1023  RESERVED
//
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
     char tempchar[13];
     StringObject tempObj;
     tempstr.toCharArray(tempchar,13);
     for ( int i = 0; i < 13; i++ ) {
       tempObj.nameLabel[i] = tempchar[i];
     }
     tempObj.nameLabel[12] = '\0';
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
//  944   958   RESERVED
//  959		    Track Selection
//  960   975   Track Power Saved State
//  976         Number of Yard Tracks
//  977   989   Yard Name
//  990   1023  RESERVED
