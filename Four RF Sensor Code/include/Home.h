#include <main.h>
#include <Hardware.h>

#ifndef HOME_H
#define HOME_H

float PID();
void reverse();
bool find_tape();
bool line_follow();

struct Home
{
    Home(int ms_before_both_high) : ms_before_both_high(ms_before_both_high) {};
    void loop();

    int state = 1;

private:
    int t_initial = 0;
    int ms_before_both_high = 0;

    void update_state(bool result);
};

extern short LINE_FOLLOW_SPEED;
// extern short max_adjustment_speed;
extern float prev_I[10], Kp, Ki, Kd, P, I, D;

static short reversed = 0;
#endif