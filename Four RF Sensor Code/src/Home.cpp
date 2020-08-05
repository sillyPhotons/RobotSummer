#include <Home.h>

short LINE_FOLLOW_SPEED = 50;

short three_on_tape_counter = 0;
short max_three_counter = 20;
float error = 0,
      prevError = 0,
      prev_I = 0,
      Kp = 4,
      Ki = 0,
      Kd = 0,
      P = 0,
      I = 0,
      D = 0;

bool seen_tape = false;

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
    P = error;
    I = error*(1./70.) + prev_I;
    D = (error - prevError)/(1./70);

    float adj = Kp * P + Ki * I + Kd * D;

    prev_I = I;
    prevError = error;

    return adj;
}

bool line_follow()
{
    short line_following_state = get_line_following_state();
    switch (line_following_state)
    {
    case 0:
        if (prevError > 0)
        {
            error = 9;
            right_motor.run_motor(35);
            left_motor.run_motor(-35);
        }
        else if (prevError < 0)
        {
            error = -9;
            right_motor.run_motor(-35);
            left_motor.run_motor(35);
        }
        // prev_I = I;
        // prevError = error;
        // return false;
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
        three_on_tape_counter += 1;
        if (three_on_tape_counter == max_three_counter)
        {
            reverse();
        }
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
        break;
    case 4:
        delay(100);
        return true;
    case 5:
        error = 0;
        three_on_tape_counter += 1;
        if (three_on_tape_counter == max_three_counter)
        {
            reverse();
        }
        // left_motor.run_motor(LINE_FOLLOW_SPEED);
        // right_motor.run_motor(LINE_FOLLOW_SPEED);
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

bool find_tape()

{
    if (check_tape() == 0 && !seen_tape)
    {
        left_motor.run_motor(45);
        right_motor.run_motor(45);
        return false;
    }
    else if (check_tape() == 0 && seen_tape)
    {
        run_both(-50, -50, 200, false);
        run_both(-55, 35, 55, false);
        seen_tape = false;
        return false;
    }

    else
    {
        seen_tape = true;
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
                run_both(-50, -50, 200, false);
                run_both(-55, 35, 55, false);
                return false;
            case 3:
                run_both(-50, -50, 200, false);
                run_both(-55, 35, 55, false);
                return false;
            case 4: // robot orientation is indeterminate
                run_both(-50, -50, 200, false);
                run_both(-55, 35, 55, false);
                return false;
            }
        }
    }
}

void reverse()
{
    right_motor.run_motor(45);
    left_motor.run_motor(-45);
    while (get_line_following_state() != 0)
    {
    }
    right_motor.run_motor(45);
    left_motor.run_motor(-45);
    while (get_line_following_state() == 0)
    {
    }
}
