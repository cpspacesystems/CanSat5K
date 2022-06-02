#ifndef DATA_H
#define DATA_H
#include "SFE_BMP180.h"

#define DATA_FRAME_SIZE 11

void data_init();
int data_calibrate();
void data_update();

float data_getAccelMag();
void data_getValues(float* arr);

void data_setLaunchTime(int mils);

float data_velocityX();
float data_AGL();

float _getPressure(SFE_BMP180 bmp);

#endif