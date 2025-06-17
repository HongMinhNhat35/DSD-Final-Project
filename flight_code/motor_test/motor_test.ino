#include <Servo.h>

Servo ESC1;
Servo ESC2;
Servo actuator;
void setup() {
  // put your setup code here, to run once:
  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  actuator.attach(6, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  delay(3000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  ESC1.write(100);
  actuator.write(60);
}
