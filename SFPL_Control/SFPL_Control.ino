
// SFPL Control V0.1
// Ron Lehmer   2023-09-27
//
// For the Arduino Uno R3
//
// Package/library dependencies for Arduino Library manager
//
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

  void set_on_time( long arg_on_time ) {
    on_time = arg_on_time;
  }

  void set_off_time( long arg_off_time ) {
    off_time = arg_off_time;
  }
  
  int state;
  int inhibit;
 private:
  long on_time;
  long off_time;
  unsigned long timer_milli;
  unsigned long timer_start;
  
};
#if 0
SFPL_Timer timer1(0,15000,30000);
SFPL_Timer timer2(0,15000,15000);
SFPL_Timer timer3(0,60000,60000);
SFPL_Timer timer4(0,60000,60000);
#endif
SFPL_Timer timers[4];

//
// Library Include Files
//


void setup() {
  
  initPins();

//  timers[0] = new SFPL_Timer(0,15000,30000);
//  timers[1] = new SFPL_Timer(0,15000,15000);
//  timers[2] = new SFPL_Timer(0,60000,60000);
//  timers[3] = new SFPL_Timer(0,60000,60000);

  timers[0].set_on_time(15000);
  timers[0].set_off_time(30000);
  timers[1].set_on_time(15000);
  timers[1].set_off_time(15000);
  timers[2].set_on_time(60000);
  timers[2].set_off_time(60000);
  timers[3].set_on_time(60000);
  timers[3].set_off_time(60000);
  
  
  delay(3000);
  Serial.begin(9600);
//  timer1.set();
}


void loop() {
  int button[4];
  int i;
  for ( i = 0 ; i < 4 ; i++ ) {
    button[i] = digitalRead(8+i);
    if ( button[i] == 0 ) {
      timers[i].set();
    }
    if ( timers[i].state == 1 ) {
      digitalWrite(4+i, HIGH);
    }
    else {
      digitalWrite(4+i, LOW);
    }
    timers[i].update();
  }
#if 0
  int button1;
  button1 = digitalRead(2);

  if ( button1 == 0 ) {
    timer1.set();
  }

  if ( timer1.state == 1 ) {
    digitalWrite(4, HIGH);
  }
  else {
    digitalWrite(4, LOW);
  }
  timer1.update();
#endif
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
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  
}
