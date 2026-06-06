//PYRO2 TRIGGER + CONTINUITY
#define CONT2 PA2
#define PYRO2_MAIN PA3

//PYRO2 TRIGGER + CONTINUITY
#define PYRO1_DROG PC5
#define CONT1 PB0

void setup() {
  // put your setup code here, to run once:
  pinMode(PYRO2_MAIN,OUTPUT);
  pinMode(PYRO1_DROG,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PYRO1_DROG,HIGH);
  digitalWrite(PYRO2_MAIN,LOW);
  delay(1000);
  digitalWrite(PYRO2_MAIN,HIGH);
  digitalWrite(PYRO1_DROG,LOW);
  delay(1000);
}
