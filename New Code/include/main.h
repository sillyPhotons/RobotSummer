#ifndef MAIN_H
#define MAIN_H

#include <Wire.h>
#include <NewPing.h>
#include <numeric>
#include <Bitmap.h>

#define SETPOINT 60
#define PGAIN PA5        // Potentiometer
#define DGAIN PA4        // Potentiometer

// Timing variables
const int start_time = HAL_GetTick();
const int TAPE_TIME = 3000;
const int TOTAL_TIME = 60000;
const int HOMING_TIME = 55000;
const int BACK_UP_TIME = 1000;

enum states
{
    entering = 1,
    searching = 2,
    homing = 3
};

#endif