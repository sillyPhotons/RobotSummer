#include <Hardware.h>

bool dumped = false;
int start_time = HAL_GetTick();
int stored_cans = 0;
volatile bool readings[4] = {false};
volatile bool crossed = false;
void Motor::run_motor(short speed)
{
    if (speed != 0)
    {
        current_speed = speed;
    }

    if (speed > 0)
    {
        pwm_stop(reverse_pin);
        speed = map(speed, 0, 100, 780, 1023);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, speed, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    else if (speed < 0)
    {
        pwm_stop(forward_pin);
        speed = map(speed, -100, 0, -1023, -780);
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, speed * -1, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    else
    {
        pwm_stop(reverse_pin);
        pwm_stop(forward_pin);
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
}

bool run_both(short left_speed, short right_speed, unsigned int ms, bool trigger_away)
{
    unsigned int current_tick = HAL_GetTick();
    left_motor.run_motor(left_speed);
    right_motor.run_motor(right_speed);
    while (HAL_GetTick() - current_tick < ms)
    {
        if (crossed && trigger_away)
        {
            crossed_tape();
            return false;
        }
    }
    left_motor.run_motor(0);
    right_motor.run_motor(0);
    return true;
}

/**
 * 0: 0 0 0 0 
 * 1: 1 0 0 0 
 * 2: 1 1 0 0 
 * 3: 1 1 1 0
 * 4: 1 1 1 1
 * 5: 0 1 1 1
 * 6: 0 0 1 1
 * 7: 0 0 0 1
 * 8: 0 1 1 0 
 * 9: 0 1 0 0
 * 10: 0 0 1 0
 * 11: ? ? ? ?
 * */
short get_line_following_state()
{
    readings[L] = analogRead(L_SENSOR) >= SETPOINT;
    readings[R] = analogRead(R_SENSOR) >= SETPOINT;
    readings[L2] = analogRead(L_SENSOR2) >= SETPOINT;
    readings[R2] = analogRead(R_SENSOR2) >= SETPOINT;

    bool L_2 = readings[L2],
         L_ = readings[L],
         R_ = readings[R],
         R_2 = readings[R2];

    if (!L_2 && !L_ && !R_ && !R_2)
    {
        return 0;
    }
    else if (L_2 && !L_ && !R_ && !R_2)
    {
        return 1;
    }
    else if (L_2 && L_ && !R_ && !R_2)
    {
        return 2;
    }
    else if (L_2 && L_ && R_ && !R_2)
    {
        return 3;
    }
    else if (L_2 && L_ && R_ && R_2)
    {
        return 4;
    }
    else if (!L_2 && L_ && R_ && R_2)
    {
        return 5;
    }
    else if (!L_2 && !L_ && R_ && R_2)
    {
        return 6;
    }
    else if (!L_2 && !L_ && !R_ && R_2)
    {
        return 7;
    }
    else if (!L_2 && L_ && R_ && !R_2)
    {
        return 8;
    }
    else if (!L_2 && L_ && !R_ && !R_2)
    {
        return 9;
    }
    else if (!L_2 && !L_ && R_ && !R_2)
    {
        return 10;
    }
    else
    {
        return 11;
    }
}

/**
 * Returns a short representing the orientation of the robot to the tape.
 * Let's imagine the tape is the x-axis, and the robot approaches the tape 
 * from the negative y axis at x = 0.
 * 
 * 0: robot is not on tape
 * 1: robot is facing the +x axis
 * 2: robot is facing the -x axis
 * 3: all sensors are on tape
 * 4: indeterminate
 * */
short get_orientation()
{
    if (check_tape() == 0)
    {
        return 0;
    }
    else
    {
        if (readings[L2] && readings[L] && readings[R] && readings[R2])
        {
            return 3;
        }
        else
        {
            if ((readings[L2] || readings[L]) && (!readings[R] || !readings[R2]))
            {
                return 1;
            }
            else if ((readings[R2] || readings[R]) && (!readings[L] || !readings[L2]))
            {
                return 2;
            }
            else
            {
                return 4;
            }
        }
    }
}

short check_tape()
{
    readings[L] = analogRead(L_SENSOR) >= SETPOINT;
    readings[R] = analogRead(R_SENSOR) >= SETPOINT;
    readings[L2] = analogRead(L_SENSOR2) >= SETPOINT;
    readings[R2] = analogRead(R_SENSOR2) >= SETPOINT;

    short count = 0;
    for (int i = 0; i < 4; i += 1)
    {
        if (readings[i])
        {
            count += 1;
        }
    }
    return count;
}

void check_sensors()
{
    readings[L] = analogRead(L_SENSOR) >= SETPOINT;
    readings[R] = analogRead(R_SENSOR) >= SETPOINT;

    if ((readings[L] || readings[R]) && !crossed)
    {
        crossed = true;
    }
}

void crossed_tape()
{
    short L_return_speed = left_motor.get_last_speed();
    short R_return_speed = right_motor.get_last_speed();

    short L_direction = -1 * L_return_speed / abs(L_return_speed);
    short R_direction = -1 * R_return_speed / abs(R_return_speed);

    if (L_direction == R_direction)
    {
        left_motor.run_motor(L_direction * 10);
        right_motor.run_motor(R_direction * 10);
    }
    else
    {
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(100);
    left_motor.run_motor(0);
    right_motor.run_motor(0);
    delay(400);

    if (L_direction == R_direction)
    {
        L_return_speed = 50;
        R_return_speed = 50;
    }
    left_motor.run_motor(L_direction * L_return_speed);
    right_motor.run_motor(R_direction * R_return_speed);
    int t = HAL_GetTick();
    while (HAL_GetTick() - t < 700)
    {
    }

    crossed = false;
    away();
}
void away()
{
    bool mot = run_both(-40, 35, map(rand()%100, 0, 100, 900, 1500), true);
    if (!mot)
    {
        return;
    }
    run_both(40, 40, 350, true);
}

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run_both(-55, 35, 20, true);
    }
    delay(500);
    bool mot = run_both(90, 90, 400, true);
    if (!mot)
    {
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP1, MICROSEC_COMPARE_FORMAT);
    mot = run_both(80, 80, 200, true);
    if (!mot)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP2, MICROSEC_COMPARE_FORMAT);
    mot = run_both(80, 80, 200, true);
    if (!mot)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(250);
    mot = run_both(80, 80, 50, true);
    pwm_start(ARM_SERVO, 50, 1300, MICROSEC_COMPARE_FORMAT);
    mot = run_both(80, 80, 50, true);
    delay(100);
    // pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    // mot = run_both(-80, -80, 300, true);
    // if (!mot)
    // {
    //     return;
    // }
    // run_both(-100, -100, 300, true);
    stored_cans += 1;
}

/**
 * Dumps the cans stored in the robot bin. Won't work if the battery isn't 
 * full
*/
void dump()
{
    int t = HAL_GetTick();
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    while (HAL_GetTick() - t < 500)
    {
    }
    // delay(500);
    run_both(0, 0, 10, false);
    t = HAL_GetTick();
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    while (HAL_GetTick() - t < 2000)
    {
    }

    // delay(3000);
    t = HAL_GetTick();

    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    while (HAL_GetTick() - t < 2000)
    {
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}