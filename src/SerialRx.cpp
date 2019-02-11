#include "SerialRx.h"

SerialRX::SerialRX() {
    newData = false;
}

bool SerialRX::recv(float *a) {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData(a);
        // showParsedData(a);
        newData = false;
        return true;
    }
    return false;
}

void SerialRX::recvWithStartEndMarkers() {
    static bool recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= NUM_CHARS) {
                    ndx = NUM_CHARS - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void SerialRX::parseData(float *a) {
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
    a[0] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[1] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[2] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[3] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[4] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[5] = atof(strtokIndx);    
    // strtokIndx = strtok(NULL, ",");
    // a[6] = atof(strtokIndx);     
}

void SerialRX::showParsedData(float *a) {
    for(int i=0; i<NUM_SERVOS; ++i) {
        Serial.print(a[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}