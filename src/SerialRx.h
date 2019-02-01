#ifndef SERIALRX_H
#define SERIALRX_H

#include "Config.h"
#include <Arduino.h> 



class SerialRX {
public:
    SerialRX();

    bool recv(float *a);

private:
    void recvWithStartEndMarkers();
    void parseData(float *a);
    void showParsedData(float *a);

    // static const byte numChars = 32;
    char receivedChars[NUM_CHARS];
    char tempChars[NUM_CHARS];
    bool newData;

};


#endif