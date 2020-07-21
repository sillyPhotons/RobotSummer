#include <main.h>
#include <Hardware.h>

#ifndef HOME_H
#define HOME_H

bool find_tape();
bool checkLine();
void PID(float L, float R);

struct Home
{
    Home(int ms_before_both_high) : ms_before_both_high(ms_before_both_high){};
    void loop();

private:
    int t_initial = 0;
    int state = 1;
    int ms_before_both_high = 0;

    void update_state(bool result);
};

extern short prevError;
extern short LINE_FOLLOW_SPEED;
extern short max_adjustment_speed;
extern float P, D, error;
extern short Lspeed;
extern short Rspeed;

#endif