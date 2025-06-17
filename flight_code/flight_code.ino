#include <Servo.h>

Servo ESC1;
Servo ESC2;
Servo rudder;
Servo actuator;

int gold = 7;

unsigned long previousMillis = 0;
int stepIndex = 0;
bool actuatorOn = false;

void setup() {
  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  rudder.attach(5, 1000, 2000);
  actuator.attach(6, 1000, 2000); // Servo: 50 to 130 degrees
  pinMode(gold, OUTPUT);
  delay(1000); // Initial pause
  ESC1.write(0);
  ESC2.write(0);
  delay(10000);
  actuator.write(60);
}

const unsigned long stepDelays[] = {
  1000,    // Step 0: Lift on
  1000,    // Step 1: Move forward
  1000,    // Step 2: Turn + actuator logic
  0,       // Step 3: Straighten rudder, stop motion
  100000,  // Step 4: Wait before repeating
};

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= stepDelays[stepIndex]) {
    previousMillis = currentMillis;

    switch (stepIndex) {
      case 0:
        ESC1.write(120);  // Lift
        break;

      case 1:
        ESC1.write(120);
        ESC2.write(100);  // Forward
        actuator.write(130); 
        break;

      case 2:
        ESC1.write(150);
        ESC2.write(120);  // Forward
        rudder.write(70); // Turn
        if (!actuatorOn) {
          actuatorOn = true;
        } else {
          actuator.write(60);
          digitalWrite(gold, HIGH);
          actuatorOn = false;
        }
        break;

      case 3:
        rudder.write(90);        // Straighten rudder
        actuator.write(130);     // Retract actuator
        digitalWrite(gold, LOW); // Turn off gold pin
        ESC2.write(0);           // Stop forward
        ESC1.write(0);           // Stop lift
        break;

      case 4:
        // Wait before next cycle
        break;
    }

    stepIndex++;
    if (stepIndex >= sizeof(stepDelays) / sizeof(stepDelays[0])) {
      stepIndex = 0;
    }
  }
}
