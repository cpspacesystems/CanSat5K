#include "data.h"
#include "log.h"
#include "error.h"

#define CALIBRATION 0
#define LAUNCH_IDLE 1
#define IN_AIR 2
#define GROUNDED 3

#define LAUNCH_THRESH 30
#define GROUND_TIME_THRESH 3000
#define GROUND_VEL_THRESH 3
#define GROUND_ALT_THRESH 800

uint32_t f_groundCheckTime;

uint8_t state;

void setup() {
    state = CALIBRATION;
    Serial.begin(9600);
    data_init();
    log_init();
}

void loop() {

    switch (state) {

    case CALIBRATION:
        if (data_calibrate() > 100) {
            state = LAUNCH_IDLE;
            Serial.println("Moving to LAUNCH_IDLE");
        }
        break;

    case LAUNCH_IDLE:
        data_update();
        if (data_getAccelMag() > LAUNCH_THRESH) {
            state = IN_AIR;
            f_groundCheckTime = millis();
            Serial.println("Moving to IN_AIR");
        }
        break;

    case IN_AIR:
        data_update();
        log_values();
        if (abs(data_velocityX()) < GROUND_VEL_THRESH && data_AGL() < GROUND_ALT_THRESH) {
            if (millis() - f_groundCheckTime > GROUND_TIME_THRESH) {
                state = GROUNDED;
                log_dumpToSD();
                Serial.println("Moving to GROUNDED");
            }
        } else {
            f_groundCheckTime = millis();
        }
        break;

    case GROUNDED:
        break;

    }
}

