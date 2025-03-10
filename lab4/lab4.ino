void setup() {
  // Left Wheels
  pinMode(0, OUTPUT);  // xIN2
  pinMode(1, OUTPUT);  // xIN1

  // Right Wheels
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  // Wait a bit before startting
  delay(10000);

  drive_in_a_straight_line(1, 150, 1.5);
  delay(3000);
  stop();
  delay(10000);
}


void loop() {

  // analogWrite(0, 150);
  // analogWrite(1, 0);
  // drive_in_a_straight_line(1, 150, 1.5);
  // delay(2000);
  // stop();
  // delay(5000);

  drive_in_a_straight_line(1, 150, 1.25);
  delay(1000);
  stop();
  delay(2500);
  turn_right(150);
  delay(1000);
  stop();
  drive_in_a_straight_line(1, 150, 1.25);
  delay(2500);
  turn_left(150);
  delay(1000);
  stop();



  // analogWrite(0, 200);  // xIN2
  // analogWrite(1, 0); // xIN1

  // analogWrite(2, 0); // xIN1
  // analogWrite(3, 200); // xIN2

  // Based on my orientation
  // // Backward PWM
  // analogWrite(0, 0);    // xIN2
  // analogWrite(1, 200);  // xIN1

  // // Forward PWM
  // analogWrite(2, 0);    // xIN1
  // analogWrite(3, 200);  // xIN2

  // Spin wheels backward
  // analogWrite(0, pwm);
  // analogWrite(1, 0);
  // analogWrite(2, pwm * calib);
  // analogWrite(3, 0);

  // drive_in_a_straight_line(1, 100, 1.20);
  // delay(5000);
  // stop();
  // delay(5000);

  // drive_in_a_straight_line(1, 150, 1.0);
  // delay(5000);
  // stop();
}

void drive_in_a_straight_line(int direction, int pwm, float calib) {
  if (direction) {  // forward

    analogWrite(0, 150);
    analogWrite(1, 0);
    analogWrite(2, 150 * 1.25);
    analogWrite(3, 0);

  } else {
    // reverse
    analogWrite(0, 0);
    analogWrite(1, 150 * 1.25);
    analogWrite(2, 0);
    analogWrite(3, 150 * 125);
  }
}

void stop() {
  analogWrite(0, 0);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}


void turn_right(int pwm) {
  analogWrite(0, pwm);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, pwm);
  delay(1000);
}

void turn_left(int pwm) {
  analogWrite(0, 0);
  analogWrite(1, pwm);
  analogWrite(2, pwm);
  analogWrite(3, 0);
  delay(1000);
}