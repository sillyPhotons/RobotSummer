#ifndef MAIN_H
#define MAIN_H

#include <Wire.h>
#include <NewPing.h>
#include <numeric>
#include <Bitmap.h>

#define SETPOINT 75
#define TAPE_TIME 3000
#define TOTAL_TIME 60000
#define HOMING_TIME 20000
#define BACK_UP_TIME 1000

enum states
{
    entering = 1,
    searching = 2,
    homing = 3
};

#endif