

// CMRS Keypad Turnout Controller
// Version v0.1
// Ron Lehmer   2021-07-10
//
// For the Arduino Uno R3
//
// Package/library dependencies for Arduino Library manager
//
// PCF8574 Library (not "PCF8574")
// LiquidCrystal I2C
//
//

//
// Library Include Files
//
#define BIGKEYPAD 1

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <PCF8574.h>
#ifdef ADAFRUIT
#include <Adafruit_MCP23017.h>
#else
#include <MCP23017.h>
#endif
#include <Wire.h>

//
// Instantiate Objects
//
PCF8574 pcf8574(0x020);
MCP23017 mcp23017 = MCP23017(0x026);
LiquidCrystal_I2C LCDisplay(0x027, 20, 4);

//
// Declare global variables and constants
//
#ifdef BIGKEYPAD
// Define the keypad pins
const byte ROWS = 4; 
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','4','7','*'},
  {'2','5','8','0'},
  {'3','6','9','#'},
  {'A','B','C','D'}
};
#else
// Define the keypad pins
const byte ROWS = 4; 
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
#endif


const int MAXYARDTRACKS = 11;

char scratchpad[3] = "00\0";
int col_count = -1;
int open_track = 0;
unsigned long interlockTime;
int interlockStatus = 0;
byte track_power[MAXYARDTRACKS];

int relay_thrown[22] = { 0, -1, 1, -1, 2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1, 10, -1 };
int relay_closed[22] = { -1, 0, -1, 1, -1, 2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1, 10 };

void scan_i2c() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if ( error == 0 ) {
      Serial.print("I2C device found at 0x");
      if ( address < 16 )
        Serial.print("0");
      Serial.print(address, HEX);
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
    Serial.println("Done\n");

  delay(5000);
}



void keypad_init() {
  pcf8574.pinMode(P0, INPUT_PULLUP);
  pcf8574.pinMode(P1, INPUT_PULLUP);
  pcf8574.pinMode(P2, INPUT_PULLUP);
  pcf8574.pinMode(P3, INPUT_PULLUP);
  pcf8574.pinMode(P4, OUTPUT, HIGH);
  pcf8574.pinMode(P5, OUTPUT, HIGH);
  pcf8574.pinMode(P6, OUTPUT, HIGH);
  pcf8574.pinMode(P7, OUTPUT, HIGH);
  
//  Serial.println("keypad_init(): Done\n");
}

void LCDisplay_init() {
  LCDisplay.clear();
  LCDisplay.backlight();
  LCDisplay.setCursor(0,0);
  LCDisplay.printstr("E Stockton\0");  
  LCDisplay.setCursor(20,0);
  LCDisplay.printstr("Power\0");
  update_scratchpad('0');
}

void recover() {
  byte temp;
  EEPROM.get(0,temp);
  Serial.print("\nEEPROM returned ");
  Serial.println(temp, DEC);
  if ( temp == 0 ) {
    line_for_mainline();
  }
  else if ( temp <= MAXYARDTRACKS ) {
    line_for_yard(temp);
  }
  else {
    EEPROM.put(0,0);
  }
  for ( int i = 0; i < MAXYARDTRACKS; i++ ) {
     EEPROM.get(i+2,temp);
     if ( temp != 0 ) {
        track_power[i] = 1; 
        track_power_display(i,1);
     }
     else {
        track_power[i] = 0;
        track_power_display(i,0);
     }
  }
}

void scan_column() {
#if 1
  col_count++;
  if ( col_count > 3 ) col_count = 0;
  if ( col_count == 0 ) {
    pcf8574.digitalWrite(P7, HIGH);
    pcf8574.digitalWrite(P4, LOW);
  }
  else if ( col_count == 1 ) {
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, LOW);    
  }
  else if ( col_count == 2 ) {
    pcf8574.digitalWrite(P5, HIGH);
    pcf8574.digitalWrite(P6, LOW);        
  }
  else if ( col_count == 3 ) {
    pcf8574.digitalWrite(P6, HIGH);
    pcf8574.digitalWrite(P7, LOW);        
  }
#endif
//  pcf8574.digitalWrite(P6, HIGH);

}

char read_keypad() {
  uint8_t row1, row2, row3, row4, row;
  static char last_char = NULL;
  char ret_val;
  static int counter = 0;
  row1 = pcf8574.digitalRead(P0);
  row2 = pcf8574.digitalRead(P1);
  row3 = pcf8574.digitalRead(P2);
  row4 = pcf8574.digitalRead(P3);
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
  LCDisplay.setCursor(14,0);
  LCDisplay.printstr(scratchpad);
}

void line_for_mainline() {
  Serial.println("\nLine for Main.");
  LCDisplayClearSecondLine();
  LCDisplay.setCursor(0,1);
  LCDisplay.printstr("Lined for Main\0)");
  update_scratchpad('0');
  update_scratchpad('0');
  EEPROM.put(0,0);
  setTurnoutsForMainline();
}

void line_for_yard(int arg_val) {
  int temp;
  if ( arg_val != -1 ) {
    temp = arg_val;
  }
  else {
    temp = atoi(scratchpad);
  }
  if ( ( temp > MAXYARDTRACKS ) || ( temp == 0 ) ) {
    Serial.print("\nError selecting track ");
    Serial.println(scratchpad);
    update_scratchpad('0');
    update_scratchpad('0');          
  }
  else {
    Serial.print("\nLining for Track ");
    Serial.println(temp, DEC);
    LCDisplayClearSecondLine();
    LCDisplay.setCursor(0,1);
    LCDisplay.printstr("Track \0");
//    LCDisplay.printstr(scratchpad);
    LCDisplay.printstr(itoa(temp,scratchpad,10));
    update_scratchpad('0');
    update_scratchpad('0');  
    EEPROM.put(0,temp); 
    setTurnoutsForTrack(temp);   
  }
}

void LCDisplayClearSecondLine() {
//  LCDisplay.setCursor(0,1);
  for (int i = 0; i < 16; i++) {
    LCDisplay.setCursor(i,1);
    LCDisplay.write(' ');
  }
}

void setTurnoutsForMainline() {

}

void setTurnoutsForTrack(int arg_track) {

}

void track_power_on() {
   int temp;
   temp = atoi(scratchpad);
   if (( temp > 0 ) && ( temp <= MAXYARDTRACKS )) {
      track_power[temp-1] = 1;
      track_power_display(temp-1,1);
      EEPROM.write(temp+1,1);
   }
   update_scratchpad('0');
   update_scratchpad('0');
}

void track_power_off() {
   int temp;
   temp = atoi(scratchpad);
   if (( temp > 0 ) && ( temp <= MAXYARDTRACKS )) {
      track_power[temp-1] = 0;
      track_power_display(temp-1,0);
      EEPROM.write(temp+1,0);
   }
   update_scratchpad('0');
   update_scratchpad('0');
}

void track_power_display(int arg_val, int arg_mode) {
   int pos1,pos2;
   if ( arg_val < 6 ) {
      pos2 = 0;
      pos1 = 28 + 2*arg_val;
   }
   else {
      pos2 = 1;
      pos1 = 28 + 2*(arg_val - 6);
   }
   LCDisplay.setCursor(pos1,pos2);
   if ( arg_mode == 0 ) {
      LCDisplay.printstr("  ");
   }
   else {
      LCDisplay.printstr(itoa(arg_val+1,scratchpad,10));
   }
}
void resetRelays() {
  for (int i = 0; i < 16; i++ )
    mcp23017.digitalWrite(i, HIGH);
  interlockStatus = 0;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nCMRS Keypad v0.4 20221117 R. Lehmer");
  Serial.println("\nI2C Scanner");
  scan_i2c();
  keypad_init();
  pcf8574.begin();
  LCDisplay.begin(16,2);
  LCDisplay_init();
  mcp23017.writeRegister(MCP23017Register::GPIO_A, 0xFF);  //Reset port A 
  mcp23017.writeRegister(MCP23017Register::GPIO_B, 0xFF);  //Reset port B
  mcp23017.portMode(MCP23017Port::A, 0);          //Port A as output
  mcp23017.portMode(MCP23017Port::B, 0);          //Port B as output
  recover();
  interlockTime = millis();
  Serial.println("\nSetup Complete");
}

void loop() {
  char keystroke;
  keystroke = read_keypad();
  if ( keystroke != NULL ) {
    Serial.print(keystroke);
    if (( keystroke == '*' ) || ( keystroke == 'D' )) {
      line_for_mainline();
    }
    else if (( keystroke == '#' ) || ( keystroke == 'C' )) {
      line_for_yard(-1);
    }
    else if ( keystroke == 'A' ) {
       track_power_on();
    }
    else if ( keystroke == 'B' ) {
       track_power_off();
    }
    else {
      update_scratchpad(keystroke);
    }
  }
  scan_column();
}
