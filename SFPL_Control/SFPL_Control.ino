
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
    _state = 0;
    inhibit = 0;
    timer_start = millis();
    timer_milli = timer_start;
  }
  
  SFPL_Timer(long arg_state, long arg_on_time, long arg_off_time) {
    _state = arg_state;
    on_time = arg_on_time;
    off_time = arg_off_time;
    timer_start = millis();
    timer_milli = timer_start;
  }
  
  ~SFPL_Timer() {
  }
  
  void set() {
    if ( ( _state == 0 ) && ( inhibit == 0 ) ) {
      timer_start = millis();      
      if ( delay_time > 0 ) 
        _state = -1;
      else
        _state = 1;
      inhibit = 1;
    } 
  }
  
  void reset() {
    _state = 0;
  }
  
  void update() {
    unsigned long timer_milli = millis();
    unsigned long differ_milli = timer_milli - timer_start;
    if (( _state == -1 ) && ( differ_milli > delay_time ) ) {
      _state = 1;
    }
    if (( _state == 1 ) && ( differ_milli > ( delay_time + on_time ) )) {
      _state = 0;
    }
    Serial.print(timer_milli);
    Serial.print(" ");
    Serial.print(timer_start);
    Serial.print(" ");
    Serial.print(_state);
    Serial.print(" ");

    if (( inhibit == 1 ) && ( differ_milli > ( delay_time + on_time + off_time )) ) {
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
  
  void set_delay_time( long arg_delay_time ) {
    delay_time = arg_delay_time;
  }

  int state() {
    int tmp;
    if ( _state == 1 )
      tmp = 1;
    else
      tmp = 0;
    return(tmp);
  }
  int _state;
  int inhibit;
 private:
  long on_time;
  long off_time;
  long delay_time;
  unsigned long timer_milli;
  unsigned long timer_start;
  
};

SFPL_Timer timers[8];

//
// Library Include Files
//


void setup() {
  
  initPins();

//  timers[0] = new SFPL_Timer(0,15000,30000);
//  timers[1] = new SFPL_Timer(0,15000,15000);
//  timers[2] = new SFPL_Timer(0,60000,60000);
//  timers[3] = new SFPL_Timer(0,60000,60000);

  timers[0].set_on_time(60000);
  timers[0].set_off_time(60000);
  timers[0].set_delay_time(0);
  timers[1].set_on_time(60000);
  timers[1].set_off_time(60000);
  timers[1].set_delay_time(1000);
  timers[2].set_on_time(60000);
  timers[2].set_off_time(60000);
  timers[2].set_delay_time(2000);
  timers[3].set_on_time(60000);
  timers[3].set_off_time(60000);
  timers[3].set_delay_time(3000);
  
  delay(3000);
  Serial.begin(9600);
//  timer1.set();
}


void loop() {
  int button[4];
  int i;
  for ( i = 0 ; i < 4 ; i++ ) {
    button[i] = digitalRead(8+i);

//    if ( button[i] == 0 ) {     // 
    if ( button[0] == 0 ) {
      timers[i].set();
    }
    if ( timers[i].state() == 1 ) {
      digitalWrite(4+i, HIGH);
    }
    else {
      digitalWrite(4+i, LOW);
    }
    Serial.print(i);
    Serial.print(" ");
    timers[i].update();
  }
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
