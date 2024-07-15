#ifndef CHRONO_H
#define CHRONO_H

#include <Arduino.h>

class chrono {
private:
    unsigned long startTime;
    unsigned long duration;
    bool running;

public:
    chrono();

    void startChrono(unsigned long duree);
    unsigned long getDelta();
    bool isFinished();
    void reset();
};

#endif