#include <Hardware.h>

short L_reading = 0;
short R_reading = 0;
volatile bool crossed = false;
volatile bool L_crossed = false;
volatile bool R_crossed = false;

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

void check_crossed_tape()
{
    L_reading = analogRead(L_SENSOR);
    R_reading = analogRead(R_SENSOR);
    if (L_reading >= SETPOINT && !L_crossed)
    {
        L_crossed = true;
    }
    if (R_reading >= SETPOINT && !R_crossed)
    {
        R_crossed = true;
    }
    if ((L_crossed || R_crossed) && !crossed)
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
    Serial1.printf("%d %d\n", L_direction, R_direction);

    left_motor.run_motor(0);
    right_motor.run_motor(0);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);

    if (L_direction == R_direction)
    {
        L_return_speed = 50;
        R_return_speed = 50;
    }
    if (L_reading >= SETPOINT || R_reading >= SETPOINT)
    {
        left_motor.run_motor(L_direction * L_return_speed);
        right_motor.run_motor(R_direction * R_return_speed);
    }
    else
    {
        left_motor.run_motor(L_direction * L_return_speed);
        right_motor.run_motor(R_direction * R_return_speed);
        int t = HAL_GetTick();
        bool left = false,
             right = false;

        if (!L_crossed)
        {
            left = true;
        }
        if (!R_crossed)
        {
            right = true;
        }
        while ((!left && !right) && HAL_GetTick() - t < 500)
        {
            if (L_reading >= SETPOINT)
            {
                left = true;
            }
            if (R_reading >= SETPOINT)
            {
                right = true;
            }
        }
    }
    delay(200);
    crossed = false;
    L_crossed = false;
    R_crossed = false;
    away();
}
void away()
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 900;
    run_both(-40, 35, turns, true);
    run_both(40, 40, 1000, true);
}

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run_both(-55, 35, 20, true);
    }
    delay(500);
    bool mot = run_both(100, 100, 450, true);
    if (!mot)
    {
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP1, MICROSEC_COMPARE_FORMAT);
    mot = run_both(100, 100, 200, true);
    if (!mot)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP2, MICROSEC_COMPARE_FORMAT);
    mot = run_both(100, 100, 200, true);
    if (!mot)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    mot = run_both(100, 100, 200, true);
    if (!mot)
    {
        return;
    }
    mot = run_both(-80, -80, 300, true);
    if (!mot)
    {
        return;
    }
    run_both(-100, -100, 300, true);
}

/**
 * Dumps the cans stored in the robot bin. Won't work if the battery isn't 
 * full
*/
void dump()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run_both(0, 0, 10, false);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}