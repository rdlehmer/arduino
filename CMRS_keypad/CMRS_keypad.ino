
// CMRS Keypad Turnout Controller
// Version v0.2
// Ron Lehmer   2023-09-03
//
// For the Arduino Uno R3/2560 Mega
//
//  Updated for 4x4 keypad testing
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
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <PCF8574.h>
#include <MCP23017.h>
#include <Wire.h>

//
// Instantiate Objects
//
PCF8574 pcf8574(0x026);
MCP23017 mcp23017(0x020);
LiquidCrystal_I2C LCDisplay(0x027, 16, 2);

//
// Declare global variables and constants
//

// Define the keypad pins
const byte ROWS = 4; 
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','4','7','*'},
  {'2','5','8','0'},
  {'3','6','9','#'},
  {'A','B','C','D'}
};

const int MAXYARDTRACKS = 10;

char scratchpad[3] = "00\0";
int col_count = -1;
int open_track = 0;
unsigned long interlockTime;
int interlockStatus = 0;

int relay_thrown[22] = { 0, -1, 1, -1, 2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1, 10, -1 };
int relay_closed[22] = { -1, 0, -1, 1, -1, 2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1, 10 };

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nCMRS Keypad v0.2 20210710 R. Lehmer");
  Serial.println("\nI2C Scanner");
  scan_i2c();
  keypad_init();
  relay_init();
  pcf8574.begin();
  LCDisplay.begin(16,2);
  LCDisplay_init();
  recover();
  interlockTime = millis();
  Serial.println("\nSetup Complete");
}


void loop() {
  char keystroke;
  keystroke = read_keypad();
  if ( keystroke != NULL ) {
    if ( keystroke == '*' ) {
      line_for_mainline();
    }
    else if ( keystroke == '#' ) {
      line_for_yard(-1);
    }
    else {
      update_scratchpad(keystroke);
    }
  }
  scan_column();
#if 0
    if ( interlockStatus == 1 ) {
    unsigned long elapsed = millis() - interlockTime;
    if ( elapsed > 500 )
      resetRelays();
  }
#endif
}

void scan_i2c() {
  byte error, address;
  int nDevices;

//  Serial.println("Scanning...");

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
  update_scratchpad('0');
}

void relay_init() {
  for (int i = 0; i < 16; i++ )
    mcp23017.pinMode(i, OUTPUT, HIGH);
}

void recover() {
  int temp;
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
}

void scan_column() {
#if 1
  col_count++;
  if ( col_count > 3 ) col_count = 0;
  if ( col_count == 0 ) {
    pcf8574.digitalWrite(P7, HIGH);
    pcf8574.digitalWrite(P6, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
    pcf8574.digitalWrite(P4, LOW);
  }
  else if ( col_count == 1 ) {
    pcf8574.digitalWrite(P7, HIGH);
    pcf8574.digitalWrite(P6, HIGH);
    pcf8574.digitalWrite(P5, LOW);
    pcf8574.digitalWrite(P4, HIGH);
  }
  else if ( col_count == 2 ) {
    pcf8574.digitalWrite(P7, HIGH);
    pcf8574.digitalWrite(P6, LOW);
    pcf8574.digitalWrite(P5, HIGH);
    pcf8574.digitalWrite(P4, HIGH); 
  }
  else if ( col_count == 3 ) {
    pcf8574.digitalWrite(P7, LOW);
    pcf8574.digitalWrite(P6, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
    pcf8574.digitalWrite(P4, HIGH);
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
      Serial.print(last_char);
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
#if 0
  mcp23017.digitalWrite(0, LOW);
  for (int i = 1; i < 16; i++ )
    mcp23017.digitalWrite(i, HIGH);
#endif
  for (int i = 0; i < 16; i++ ) {
    if ( relay_thrown[i] == 0 )
      mcp23017.digitalWrite(i, LOW);
    if ( relay_closed[i] > 0 )
      mcp23017.digitalWrite(i, LOW);
  }
  interlockTime = millis();
  interlockStatus = 1;
}

void setTurnoutsForTrack(int arg_track) {
#if 0
  mcp23017.digitalWrite(0, HIGH);
  for (int i = 1; i < 16; i++ ) {
    if ( i == arg_track )
      mcp23017.digitalWrite(i, LOW);
    else
      mcp23017.digitalWrite(i, HIGH);
  }
#endif
  for (int i = 0; i < 16; i++ ) {
    if ( relay_thrown[i] == arg_track )
      mcp23017.digitalWrite(i, LOW);
    if ( ( relay_closed[i] >= 0 ) && ( relay_closed[i] != arg_track ) )
      mcp23017.digitalWrite(i, LOW);
  }
  interlockTime = millis();
  interlockStatus = 1;
}

void resetRelays() {
  for (int i = 0; i < 16; i++ )
    mcp23017.digitalWrite(i, HIGH);
  interlockStatus = 0;
}
