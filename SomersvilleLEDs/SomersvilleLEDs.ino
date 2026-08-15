int ifRead0, ifRead1, ifRead2;

int T32 = 4;
int T33 = 5;
int T34 = 6;

int LED10 = 11;
int LED11 = 12;
int LED13 = 13;
int LED10A = 8;
int LED11A = 9;
int LED13A = 10;

void setup() {
  Serial.begin (9600); 
  pinMode (T32, INPUT_PULLUP);
  pinMode (T33, INPUT_PULLUP);
  pinMode (T34, INPUT_PULLUP);
  pinMode (LED10, OUTPUT);
  pinMode (LED11, OUTPUT);
  pinMode (LED13, OUTPUT);
  pinMode (LED10A, OUTPUT);
  pinMode (LED11A, OUTPUT);
  pinMode (LED13A, OUTPUT);
}

void loop() {

ifRead0 = 0;  
ifRead1 = 0;      
ifRead2 = 0;
for (int i = 0;  i <= 100; i++) {
ifRead0 = ifRead0 + digitalRead(T32);  
ifRead1 = ifRead1 + digitalRead(T33);      
ifRead2 = ifRead2 + digitalRead(T34);
  }
 
if (ifRead0 > 70) {ifRead0 = 1;}
   else {ifRead0 = 0;}
if (ifRead1 > 70) {ifRead1 = 1;}
   else {ifRead1 = 0;}
if (ifRead2 > 70) {ifRead2 = 1;}
   else {ifRead2 = 0;}      
Serial.println(ifRead1);      

//Pittsburg  Route
if ((ifRead0 == LOW) && (ifRead1 == HIGH)) {
  digitalWrite(LED13, HIGH);
  digitalWrite(LED10, LOW);
  digitalWrite(LED11, LOW);
  digitalWrite(LED13A, HIGH);
  digitalWrite(LED10A, LOW);
  digitalWrite(LED11A, LOW);}
else {
//  Crossover  route
if ((ifRead1 == LOW) && (ifRead2 == LOW)) {
  digitalWrite(LED11, HIGH);
  digitalWrite(LED10, LOW);
  digitalWrite(LED13, LOW);
  digitalWrite(LED11A, HIGH);
  digitalWrite(LED10A, LOW);
  digitalWrite(LED13A, LOW);}
  else {
 // Somersville Route   
if ((ifRead0 == HIGH) && (ifRead2 == HIGH)) {
  digitalWrite(LED10, HIGH);
  digitalWrite(LED11, LOW);
  digitalWrite(LED13, LOW);
  digitalWrite(LED10A, HIGH);
  digitalWrite(LED11A, LOW);
  digitalWrite(LED13A, LOW);}
  else {
  digitalWrite(LED10, LOW);
  digitalWrite(LED11, LOW);
  digitalWrite(LED13, LOW);
  digitalWrite(LED10A, LOW);
  digitalWrite(LED11A, LOW);
  digitalWrite(LED13A, LOW);}    
  }

}

}
