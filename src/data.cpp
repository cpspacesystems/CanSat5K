#include "data.h"
#include <Wire.h>
#include "BMI088.h"
#include "Adafruit_BMP3XX.h"
#include "error.h"

#define PIN_GEIGER1 23
#define PIN_GEIGER2 25
#define PIN_METER A2

#define METER_FS 185
#define GEIGER_SAMPLE_TIME 10000
#define CPM2USV 220.0f // clicks per minute

SFE_BMP180 bmp1(Wire);

Adafruit_BMP3XX baro;

Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

const float u_groundLevel = 103.632f;
const float u_altitudeConstant = pow(1 - u_groundLevel / 44330.0, -5.2549) / 100.0;

float c_groundPressure;
float c_seaPressure;
int c_calibrations;

float f_accelX;
float f_accelY;
float f_accelZ;
float f_accelMag;
float f_gyroX;
float f_gyroY;
float f_gyroZ;
float f_pressure;
float f_AGL;
float f_ASL;
float f_prevAGL;
float f_velocityX;

float f_pressure1;
int f_geigerTimer;
int f_geigerTicks1;
int f_geigerTicks2;
float f_radiation; // In nano seiverts per hour (I think)
int f_launchTime;

int f_newTime;
int f_oldTime;
float f_deltaTime;

volatile int v_geigerTicks1 = 0;
volatile int v_geigerTicks2 = 0;

void pin2irq() {
    v_geigerTicks1++;
}

void pin3irq() {
    v_geigerTicks2++;
}

void data_init() {

    //Wire1.setSCL(16);
    //Wire1.setSDA(17);
    if (!bmp1.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring");
    }

    Serial.println("INIT: baro");
    error_assert(baro.begin_I2C(0x76), 1, BARO_ERR_OFFSET, "baro.begin_I2C");
    error_assert(baro.setTemperatureOversampling(BMP3_NO_OVERSAMPLING), 1, BARO_ERR_OFFSET, "baro.setTemperatureOversampling");
    error_assert(baro.setPressureOversampling(BMP3_OVERSAMPLING_8X), 1, BARO_ERR_OFFSET, "baro.setPressureOversampling");
    error_assert(baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15), 1, BARO_ERR_OFFSET, "baro.setIIRFilterCoeff");
    error_assert(baro.setOutputDataRate(BMP3_ODR_200_HZ), 1, BARO_ERR_OFFSET, "baro.setOutputDataRate");

    Serial.println("INIT: Accel");

    error_assert(accel.begin(), 1, ACCEL_ERR_OFFSET, "accel.begin()");
    error_assert(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ), 1, ACCEL_ERR_OFFSET, "accel.setOdr()");
    error_assert(accel.setRange(Bmi088Accel::RANGE_12G), 1, ACCEL_ERR_OFFSET, "accel.setRange()");
    
    Serial.println("INIT: Gyro");

    error_assert(gyro.begin(), 1, GYRO_ERR_OFFSET, "gyro.begin()");
    error_assert(gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ), 1, GYRO_ERR_OFFSET, "gyro.setOdr()");
    error_assert(gyro.setRange(Bmi088Gyro::RANGE_1000DPS), 1, GYRO_ERR_OFFSET, "gyro.setRange()");

    cli();

    attachInterrupt(PIN_GEIGER1, pin2irq, RISING);
    attachInterrupt(PIN_GEIGER2, pin3irq, RISING);

    sei();

    pinMode(PIN_METER, OUTPUT);
    analogWrite(PIN_METER, 182);

    f_geigerTimer = millis();

    c_calibrations = 0;

    f_newTime = millis();
    f_oldTime = millis();
}

int data_calibrate() {
    baro.performReading();
    gyro.readSensor();

    if (c_calibrations == 0) {
        c_groundPressure = baro.readPressure();
    
    } else {
        c_groundPressure += (baro.readPressure() - c_groundPressure) / c_calibrations;
        c_seaPressure = c_groundPressure * u_altitudeConstant;
    }

    c_calibrations++;
    return c_calibrations;
}

void data_update() {

    f_newTime = millis();
    f_deltaTime = (f_newTime - f_oldTime) / 1000.0f;
    f_oldTime = f_newTime;

    f_pressure1 = _getPressure(bmp1);

    accel.readSensor();
    f_accelX = accel.getAccelX_mss();
    f_accelY = accel.getAccelY_mss();
    f_accelZ = accel.getAccelZ_mss();

    f_accelMag = sqrt(sq(f_accelX) + sq(f_accelY) + sq(f_accelZ));

    gyro.readSensor();
    f_gyroX = gyro.getGyroX_rads();
    f_gyroY = gyro.getGyroY_rads();
    f_gyroZ = gyro.getGyroZ_rads();

    if (millis() - f_geigerTimer > GEIGER_SAMPLE_TIME) {
        cli();
        f_geigerTicks1 = v_geigerTicks1;
        f_geigerTicks2 = v_geigerTicks2;
        v_geigerTicks1 = 0;
        v_geigerTicks2 = 0;
        sei();

        f_radiation = (f_geigerTicks1 + f_geigerTicks2) / 2.0f * (60000.0f / GEIGER_SAMPLE_TIME) / CPM2USV;

        analogWrite(PIN_METER, (f_geigerTicks1 + f_geigerTicks2) * (METER_FS/100.0f));

        f_geigerTimer = millis();
        Serial.println(f_radiation);

    }

    baro.performReading();
    f_pressure = baro.readPressure();
    f_ASL = baro.readAltitude(c_seaPressure);
    f_AGL = f_ASL - u_groundLevel;

    if (c_calibrations > 10)  {
        f_velocityX = (f_AGL - f_prevAGL) / f_deltaTime;
    }

    f_prevAGL = f_AGL;

}

float data_getAccelMag() {
    return f_accelMag;
}

void data_getValues(float* arr) {
    arr[0] = (float)(millis() - f_launchTime);

    arr[1] = f_pressure1;
    arr[2] = f_radiation;

    arr[3] = f_accelX;
    arr[4] = f_accelY;
    arr[5] = f_accelZ;

    arr[6] = f_gyroX;
    arr[7] = f_gyroY;
    arr[8] = f_gyroZ;

    arr[9] = f_pressure;
    arr[10] = f_AGL;
}

void data_setLaunchTime(int mils) {
    f_launchTime = mils;
}

float data_velocityX() {
    return f_velocityX;
}

float data_AGL() {
    return f_AGL;
}

void data_log() {
    Serial.println(f_pressure1);
}

float _getPressure(SFE_BMP180 bmp) {
    char status;
    double T, P;

    status = bmp.startTemperature();
    if (status != 0) {
        delay(status);
    } else {
        Serial.println("startTemp");
        return -1.0;
    }

    status = bmp.getTemperature(T);
    if (status == 0) {
        Serial.println("getTemp");
        return -1.0;
    }

    status = bmp.startPressure(3);
    if (status != 0) {
        delay(status);
    } else {
        Serial.println("startPressure");
        return -1.0;
    }

    status = bmp.getPressure(P, T);
    if (status == 0) {
        Serial.println("getPressure");
        return -1.0;
    }

    return P;
}