#include <Arduino.h> 
#include "Config.h"
#include "ServoController.h"
#include "SerialRx.h"

// const byte numChars = 32;
// char receivedChars[numChars];
// char tempChars[numChars];

ServoController* sc;
SerialRX* sRX;
float angles[NUM_SERVOS];
// boolean newData = false;

//boom here it is 

void setup() {
    Serial.begin(9600);
    Serial.println("Initialising Robot Arm");
    sc = new ServoController();
    delay(50);
}

void setServoPos() {
    for(int i=0; i<NUM_SERVOS; ++i) {
        sc->setAngle(angles[i],i);
    }
}


void loop() {
    if(sRX->recv(angles)) {
        setServoPos();
    }


    delay(100);
}