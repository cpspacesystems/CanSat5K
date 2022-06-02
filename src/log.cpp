#include <Arduino.h>
#include "SPIFlash.h"
#include "data.h"
#include "SD.h"
#include "log.h"
#include "error.h"

#define PIN_FLASH 31
#define FLASH_CAPACITY 0x1000000
#define SD_PIN 26
#define CSV_DELIMITER ','

uint32_t start_pointer = 0;
uint32_t end_pointer = 0;
uint32_t f_frameIndex = 0;

File logFile;


SPIFlash flash(PIN_FLASH);

union data_frame {
    float f[DATA_FRAME_SIZE];
    byte b[DATA_FRAME_SIZE * sizeof(float)];
};

void log_init() {

    Serial.println("INIT: Flash");

    error_assert(flash.initialize(), true, FLASH_ERR_OFFSET, "flash.initialize()");

    //flash.chipErase();
    while (flash.busy()) {}

    Serial.println("INIT: SD");

    error_assert(SD.begin(SD_PIN), true, SD_ERR_OFFSET, "SD.begin()");

}

void log_values() {
    
    data_frame frame;

    data_getValues(frame.f);
    flash.writeBytes(end_pointer, frame.b, DATA_FRAME_SIZE * sizeof(float));
    end_pointer += DATA_FRAME_SIZE * sizeof(float);

    f_frameIndex++;

}

void log_dumpToSD() {

    // Search for a unique file name on the SD card

    int fileNum = 0;
    char name[11];
    char num[3];
    while (fileNum < 100) {
        name[0] = 0;
        num[0] = 0;
        itoa(fileNum, num, 10);
        strcat(name, "log_");
        strcat(name, num);
        strcat(name, ".csv");
        Serial.print("Tried file name ");
        Serial.println(name);
        if (!SD.exists(name)) break;
        fileNum++;
    }

    // Open file and log column labels
    logFile = SD.open(name, FILE_WRITE);
    logFile.println("MET,pressure1,pressure2,radiation,accelX,accelY,accelZ,pressure,AGL");

    uint32_t dataAddr = start_pointer;
    data_frame frame;

    uint32_t pointer = start_pointer;
    
    for (uint16_t i = 0; i < f_frameIndex; i += 1) {

        
        flash.readBytes(pointer, frame.b, DATA_FRAME_SIZE * sizeof(float));
        pointer += DATA_FRAME_SIZE * sizeof(float);
        
        for (int j = 0; j < DATA_FRAME_SIZE; j++) {
            logFile.print(frame.f[j]);
            logFile.print(CSV_DELIMITER);
        }

        logFile.println();

    }

    logFile.close();

}