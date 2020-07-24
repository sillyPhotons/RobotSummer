#include <Hardware.h>

void Motor::run_motor(int speed)
{
    pwm_stop(reverse_pin);
    pwm_stop(forward_pin);
    if (speed > 0)
    {
        speed = map(speed, 0, 100, 780, 1023);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, speed, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    else if (speed < 0)
    {
        speed = map(speed, -100, 0, -1023, -780);
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, speed * -1, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    else
    {
        pwm_start(reverse_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(forward_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
}

bool run_both(short left_speed, short right_speed, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    left_motor.run_motor(left_speed);
    right_motor.run_motor(right_speed);

    while (HAL_GetTick() - current_tick < ms)
    {
        if ((left_speed > 0 && right_speed > 0) || (left_speed < 0 && right_speed < 0))
        {
            short L = analogRead(L_SENSOR);
            short R = analogRead(R_SENSOR);

            if (L >= SETPOINT || R >= SETPOINT)
            {
                if (left_speed > 0 && right_speed > 0)
                {
                    left_motor.run_motor(0);
                    right_motor.run_motor(0);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    // pwm_stop(ARM_SERVO);

                    // take measurement again/,.
                    L = analogRead(L_SENSOR);
                    R = analogRead(R_SENSOR);

                    if (L >= SETPOINT || R >= SETPOINT)
                    {
                        left_motor.run_motor(-55);
                        right_motor.run_motor(-55);
                        delay(200);
                        away();
                    }
                    else
                    {
                        left_motor.run_motor(-55);
                        right_motor.run_motor(-55);
                        while (analogRead(L_SENSOR) < SETPOINT || analogRead(R_SENSOR < SETPOINT))
                        {
                        }
                        delay(200);
                        away();
                    }
                    return false;
                }
                else
                {
                    left_motor.run_motor(10);
                    right_motor.run_motor(10);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    left_motor.run_motor(80);
                    right_motor.run_motor(80);
                    delay(200);
                    // pwm_stop(ARM_SERVO);
                    away();
                    return false;
                }
            }
        }
    }
    left_motor.run_motor(0);
    right_motor.run_motor(0);
    return true;
}

void away()
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 800;
    left_motor.run_motor(-55);
    right_motor.run_motor(35);
    while (HAL_GetTick() - t < turns)
    {
    }
    run_both(40, 40, 1000);
}

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run_both(-55, 35, 50);
    }
    delay(600);

    bool mot_1 = run_both(100, 100, 500);
    if (!mot_1)
    {
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_2 = run_both(50, 50, 300);
    if (!mot_2)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_3 = run_both(50, 50, 200);
    if (!mot_3)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    bool mot_4 = run_both(-80, -80, 200);
    if (!mot_4)
    {
        return;
    }
    run_both(-100, -100, 300);
}

/**
 * Dumps the cans stored in the robot bin. Won't work if the battery isn't 
 * full
*/
void dump()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run_both(0, 0, 10);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}