
// SFPL Control V0.1
// Ron Lehmer   2023-09-27
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

class SFPL_Timer {
 public:
  SFPL_Timer() {
    state = 0;
    inhibit = 0;
    timer_start = millis();
    timer_milli = timer_start;
  }
  
  SFPL_Timer(int arg_state, int arg_on_time, int arg_off_time) {
    state = arg_state;
    on_time = arg_on_time;
    off_time = arg_off_time;
    timer_start = millis();
    timer_milli = timer_start;
  }
  
  ~SFPL_Timer() {
  }
  
  void set() {
    if ( ( state != 1 ) && ( inhibit == 0 ) ) {
      timer_start = millis();      
      state = 1;
      inhibit = 1;
    } 
  }
  
  void reset() {
    state = 0;
  }
  
  void update() {
    unsigned long timer_milli = millis();
    unsigned long differ_milli = timer_milli - timer_start;
    if (( state == 1 ) && ( differ_milli > on_time )) {
      state = 0;
    }
    Serial.print(timer_milli);
    Serial.print(" ");
    Serial.print(timer_start);
    Serial.print(" ");
    Serial.print(state);
    Serial.print(" ");

    if (( inhibit == 1 ) && ( differ_milli > ( on_time + off_time )) ) {
      inhibit = 0;
    }
    Serial.println(inhibit);
  }
  
  int state;
  int inhibit;
 private:
  long on_time;
  long off_time;
  unsigned long timer_milli;
  unsigned long timer_start;
  
};

SFPL_Timer timer1(0,15000,30000);
//
// Library Include Files
//


void setup() {
  initPins();
  delay(3000);
  Serial.begin(9600);
//  timer1.set();
}


void loop() {
  int button1;
  button1 = digitalRead(2);
  if ( button1 == 0 ) {
    timer1.set();
  }
#if 0  
  while ( timer1.state == 1 ) {
    digitalWrite(4, HIGH);
    timer1.update();
  }
  digitalWrite(4, LOW);
#endif
  if ( timer1.state == 1 ) {
    digitalWrite(4, HIGH);
  }
  else {
    digitalWrite(4, LOW);
  }
  timer1.update();
}

void initPins() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  
}
