#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CommandLine.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

char movementType = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("16 channel Servo test!");
    
    pwm.begin();
    
    pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
    
    //yield();
}

// the code inside loop() has been updated by Robojax
void loop() {
  if (Serial.available()) {
    movementType = Serial.read();
    Serial.println(movementType);
  }
  while (Serial.available()) {
    Serial.read();
  }
  if (movementType == '0') {
    defaultPos();
  } else if (movementType == '1') {
    updown();
  } else if (movementType == '2') {
    rotate();
  } else {
    defaultPos();
  }
}


void defaultPos() {
    pwm.setPWM(0, 0, 300);
    pwm.setPWM(2, 0, 300);
    pwm.setPWM(4, 0, 300);
    pwm.setPWM(1, 0, 300);
    pwm.setPWM(3, 0, 300);
    pwm.setPWM(5, 0, 300);
    delay(2000);
}

void updown() {
    for (uint16_t pulselen = 200; pulselen < 400; pulselen++) {
      pwm.setPWM(0, 0, pulselen);
      pwm.setPWM(2, 0, pulselen);
      pwm.setPWM(4, 0, pulselen);
      pwm.setPWM(1, 0, SERVOMAX-pulselen);
      pwm.setPWM(3, 0, SERVOMAX-pulselen);
      pwm.setPWM(5, 0, SERVOMAX-pulselen);
    }
    delay(2000);
    for (uint16_t pulselen = 400; pulselen > 200; pulselen--) {
      pwm.setPWM(0, 0, pulselen);
      pwm.setPWM(2, 0, pulselen);
      pwm.setPWM(4, 0, pulselen);
      pwm.setPWM(1, 0, SERVOMAX-pulselen);
      pwm.setPWM(3, 0, SERVOMAX-pulselen);
      pwm.setPWM(5, 0, SERVOMAX-pulselen);
    }
    delay(2000);
}

void rotate() {
    for (uint16_t pulselen = 200; pulselen < 400; pulselen++) {
      pwm.setPWM(0, 0, pulselen);
      pwm.setPWM(2, 0, pulselen);
      pwm.setPWM(4, 0, pulselen);
      pwm.setPWM(1, 0, pulselen);
      pwm.setPWM(3, 0, pulselen);
      pwm.setPWM(5, 0, pulselen);
    }
    delay(2000);
        for (uint16_t pulselen = 400; pulselen > 200; pulselen--) {
      pwm.setPWM(0, 0, pulselen);
      pwm.setPWM(2, 0, pulselen);
      pwm.setPWM(4, 0, pulselen);
      pwm.setPWM(1, 0, pulselen);
      pwm.setPWM(3, 0, pulselen);
      pwm.setPWM(5, 0, pulselen);
    }
    delay(2000);
}
