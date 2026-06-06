#define LED PA0

void setup() {
  delay(3000);
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED ON");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED OFF");
  delay(1000);
}
