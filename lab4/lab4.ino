void setup() {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}

void loop() {

  // Forward
  analogWrite(0, 200);
  analogWrite(1, 0);

  analogWrite(2, 200);
  analogWrite(3, 0);

  delay(5000);

  analogWrite(0, 0);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}