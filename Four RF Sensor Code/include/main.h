#ifndef MAIN_H
#define MAIN_H

#include <Wire.h>
#include <NewPing.h>
#include <numeric>
#include <Bitmap.h>

#define SETPOINT 60
#define TEST_SETPOINT 250
#define TAPE_TIME 2500
#define TOTAL_TIME 60000
#define HOMING_TIME 60000
#define BACK_UP_TIME 1000


enum C_states
{
    entering = 1,
    searching = 2,
    homing = 3
};

enum E_states
{
    spin = 1,
    retrieve = 2,
    complete = 3
};

#endif