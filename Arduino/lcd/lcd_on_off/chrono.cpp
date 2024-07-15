#include "chrono.h"

chrono::chrono() : startTime(0), duration(0), running(false) {}

void chrono::startChrono(unsigned long duree) {
    startTime = millis();
    duration = duree;
    running = true;
}

bool chrono::isFinished() {
    if (running && (millis() - startTime >= duration)) {
        running = false;
        return true;
    }
    return false;
}

void chrono::reset() {
    startTime = millis();
    running = true;
}

unsigned long chrono::getDelta() {
    unsigned long delta = millis()-startTime;
    return delta;
}
