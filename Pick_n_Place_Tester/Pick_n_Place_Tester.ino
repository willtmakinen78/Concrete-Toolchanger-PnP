#define POT_PIN 0
#define PUMP_PIN 3


void setup() {
  Serial.begin(9600);

}

void loop() {
  

  int potVal = analogRead(POT_PIN);
  int pumpVal = map(potVal, 0, 1023, 0, 255);
  analogWrite(PUMP_PIN, pumpVal);

  Serial.println(pumpVal);
}
