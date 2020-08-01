#include <Home.h>

short prevError = 0;
short LINE_FOLLOW_SPEED = 33;
float P, D, error = 0;
short Lspeed = 30,
      Rspeed = 30;

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
        update_state(checkLine());
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
        L_crossed = false;
        R_crossed = false;
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

/**
 * Checks if robot sees the line or not.
 * If yes, PID line following.
 * If no, sharp turn based on previous location to find the line
 * @returns: true if complete
*/
bool checkLine()
{
    short L = L_reading;
    short R = R_reading;

    if (L > SETPOINT)
    {
        L = SETPOINT;
    }
    if (R > SETPOINT)
    {
        R = SETPOINT;
    }
    if (L < SETPOINT && R < SETPOINT)
    {
        if (((L / R > 2.0) || (R / L > 2.0)) && ((L > 250) || R > 250))
        {
            PID(L, R);
        }

        else
        {
            if (prevError > 0)
            {
                right_motor.run_motor(LINE_FOLLOW_SPEED);
                left_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
            }
            else if (prevError < 0)
            {
                right_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
                left_motor.run_motor(LINE_FOLLOW_SPEED);
            }
        }
    }
    // ONLY FOR FOR ENDING
    else if (L == SETPOINT && R == SETPOINT)
    {
        return true;
    }
    else
    {
        PID(L, R);
        return false;
    }
    return false;
}

/**
 * Computes the adjustment for the motors based on how far off the sensors
 * are from the line and changes motor speed.
 * @param L analog reading from left TCRT5000
 * @param R analog reading from right TCRT5000
*/
void PID(float L, float R)
{

    float R_diff = abs(R - SETPOINT) / ((R + SETPOINT) / 2.0);
    float L_diff = abs(L - SETPOINT) / ((L + SETPOINT) / 2.0);

    if (R_diff + L_diff < 0.10)
    {
        error = 0.0;
    }
    else
    {
        if (R_diff > 0.05 && L_diff < 0.05)
        {
            error = -1;
        }
        else if (R_diff < 0.05 && L_diff > 0.05)
        {
            error = 1;
        }
        else
        {
            error = (prevError / abs(prevError)) * 5;
        }
    }
    prevError = error;
    right_motor.run_motor(Rspeed);
    left_motor.run_motor(Lspeed);
}

bool find_tape()
{
    if (L_reading < SETPOINT && R_reading < SETPOINT)
    {
        left_motor.run_motor(35);
        right_motor.run_motor(35);
        return false;
    }

    else
    {
        while (true)
        {
            if (L_reading >= SETPOINT)
            {
                if (R_reading < SETPOINT)
                {
                    break;
                }
            }
            else if (R_reading >= SETPOINT)
            {
                if (L_reading < SETPOINT)
                {
                    break;
                }
            }
            run_both(0, 0, 75, false);
            run_both(-55, 35, 50, false);
            run_both(0, 0, 75, false);
        }
        left_motor.run_motor(0);
        right_motor.run_motor(0);
        return true;
    }
}
