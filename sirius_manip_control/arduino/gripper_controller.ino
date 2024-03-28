#define PWM_PIN 9

#include <Servo.h>

Servo myservo;  
void setup() {
  Serial.begin(9600);
}


int value = 100;
int timeoutCount = 0;

void loop() {
  while (Serial.available()) {
    value = Serial.read();

    myservo.attach(PWM_PIN, 0, 255);
    myservo.write(value);
    timeoutCount = 0;
  }

  if (timeoutCount > 50) {
    myservo.detach();
  }

  delay(10);
  timeoutCount++;
}