//PYRO2 TRIGGER + CONTINUITY
#define CONT2 PA2
#define PYRO2_MAIN PA3

//PYRO2 TRIGGER + CONTINUITY
#define PYRO1_DROG PC5
#define CONT1 PB0

#define VBAT PB1

void setup() {
  // put your setup code here, to run once:
  pinMode(PYRO2_MAIN,OUTPUT);digitalWrite(PYRO2_MAIN,0);
  pinMode(PYRO1_DROG,OUTPUT);digitalWrite(PYRO1_DROG,0);
  pinMode(CONT1,INPUT); pinMode(CONT2,INPUT); pinMode(VBAT,INPUT);

  delay(3000);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  String str = "CONT1: " + String(analogRead(CONT1)) + ", CONT2: " + String(analogRead(CONT2)) + ", VBAT = " + String(1.285 * 3.30 * (float)(analogRead(VBAT)) / 1023.0);
  Serial.println(str);
  delay(500);
}
