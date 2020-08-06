#include <Home.h>

short LINE_FOLLOW_SPEED = 40;
int three_on_tape_time = 0;
short integeral_counter = 0;
short max_3_on_tape_time = 1000;
float error = 0,
prevError = 0,
prev_I[10] ={ 0 },
Kp = 0.23,
Ki = 0.0,
Kd = 0.5,
P = 0,
I = 0,
D = 0;

void Home::loop()
{
    switch (state)
    {
    case 1: // looking for tape
    {
        update_state(find_tape());
        break;
    }
    case 2: // found tape, homing
    {
        update_state(line_follow());
        break;
    }

    case 3: // both sensors high
    {
        unsigned int t = HAL_GetTick();
        left_motor.run_motor(-100);
        right_motor.run_motor(-100);
        while ((HAL_GetTick() - t) < BACK_UP_TIME)
        {
        }
        left_motor.run_motor(0);
        right_motor.run_motor(0);
        update_state(true);
        break;
    }

    case 4: // dumping
    {
        crossed = false;
        dump();
        left_motor.run_motor(0);
        right_motor.run_motor(0);
        update_state(true);
        break;
    }

    case 5: // complete
        // dumped = true;
        update_state(true);
        break;
    }
}
void Home::update_state(bool result)
{
    switch (state)
    {
    case 1: // looking for tape
        if (result)
        {
            state = 2;
            t_initial = HAL_GetTick();
        }
        break;

    case 2: // found tape, homing
        if (HAL_GetTick() - t_initial >= ms_before_both_high && result)
        {
            state = 3;
        }
        else if (reversed == 1 && result == false) {
            reversed = 2;
            state = 1;
        }
        else
        {
            state = 2;
        }
        break;

    case 3: // both sensors high
        state = 4;
        break;

    case 4: // dumping
        state = 5;
        break;
    case 5:
        break;
    }
}

float PID()
{
    short E = error;

    prev_I[integeral_counter] = E;
    if (integeral_counter < 10)
    {
        integeral_counter += 1;
    }
    else
    {
        integeral_counter = 0;
    }

    P = E;
    I = 0;
    for (int i = 0; i < 10; i += 1)
    {
        I += prev_I[i];
    }
    D = error - prevError;

    float adj = Kp * P + Ki * I + Kd * D;

    prevError = error;
    return adj;
}

bool line_follow()
{
    short line_following_state = get_line_following_state();
    // Serial1.printf("%d\n",line_following_state);
    switch (line_following_state)
    {
    case 0:
        if (prevError > 0)
        {
            error = 5;
            right_motor.run_motor(35);
            left_motor.run_motor(-35);
        }
        else if (prevError < 0)
        {
            error = -5;
            right_motor.run_motor(-35);
            left_motor.run_motor(35);
        }
        prevError = error;
        return false;
        break;
    case 1:
        error = -4;
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // prevError = error;
        break;
    case 2:
        error = -3;
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // prevError = error;
        break;
    case 3:
        error = 0;
        if (three_on_tape_time == 0) {
            three_on_tape_time = HAL_GetTick();
        }
        else if (HAL_GetTick() - three_on_tape_time > max_3_on_tape_time && reversed == 0) {
            reverse();
        }
        break;
    case 4:
        delay(100);
        return true;
    case 5:
        error = 0;
        if (three_on_tape_time == 0) {
            three_on_tape_time = HAL_GetTick();
        }
        else if (HAL_GetTick() - three_on_tape_time > max_3_on_tape_time && reversed == 0) {
            reverse();
        }
        break;
    case 6:
        error = 3;
        // left_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
        // prevError = error;
        break;
    case 7:
        error = 4;
        // left_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
        // prevError = error;
        break;
    case 8:
        error = 0;
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
        break;
    case 9:
        error = -1;
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // prevError = error;
        break;
    case 10:
        error = 1;
        // left_motor.run_motor(LINE_FOLLOW_SPEED + 5);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
        // prevError = error;
        break;
    }
    float adj = PID();

    int left_motor_speed = constrain(LINE_FOLLOW_SPEED - adj, -80, 80);
    int right_motor_speed = constrain(LINE_FOLLOW_SPEED + adj, -80, 80);

    left_motor.run_motor(left_motor_speed);
    right_motor.run_motor(right_motor_speed);
    // Serial1.printf("%d %d\n", right_motor_speed, left_motor_speed);

    return false;
}

/**
 * true for left, false for right
 * */
bool find_tape()

{
    if (check_tape() == 0)
    {
        left_motor.run_motor(45);
        right_motor.run_motor(45);
        return false;
    }

    else
    {
        while (true)
        {
            int orientation = get_orientation();

            switch (orientation)
            {
            case 1: // robot facing +x
                right_motor.run_motor(30);
                left_motor.run_motor(30);
                while (get_line_following_state() != 0)
                {
                }
                right_motor.run_motor(33);
                left_motor.run_motor(0);
                while (get_line_following_state() == 0)
                {
                }
                return true;
            case 2:
                run_both(-50, -50, 500, false);
                run_both(-55, 35, 100, false);
                return false;
            case 3:
                run_both(-50, -50, 500, false);
                run_both(-55, 35, 100, false);
                return false;
            case 4: // robot orientation is indeterminate
                run_both(-50, -50, 500, false);
                run_both(-55, 35, 100, false);
                return false;
            }
        }
    }
}

void reverse()
{
    run_both(-40, 40, 300, false);
    reversed = 1;
}
