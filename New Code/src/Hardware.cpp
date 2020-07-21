#include <Hardware.h>

void Motor::run_motor(int speed)
{
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

/**
 * Run a two motor at the given speed for the given number of milliseconds
 * @param motor1, motor2: motor objects
 * @param speed1, speed2: speed in [-100, 100]
 * @param ms: number of milliseconds the motors will run for
 * @returns: a boolean. True if the action was completed, false if it was
 * interrupted
 * 
 * 
 * TODO: Due to momentum, the robot can't stop right away when it passes tape.
 *       What can we do?
*/
bool run2_for_ms(Motor *motor1, Motor *motor2, short speed1, short speed2, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor1->run_motor(speed1);
    motor2->run_motor(speed2);
    while (HAL_GetTick() - current_tick < ms)
    {
        if ((speed1 > 0 && speed2 > 0) || (speed1 < 0 && speed2 < 0))
        {
            short L = analogRead(L_SENSOR);
            short R = analogRead(R_SENSOR);

            if (L > SETPOINT || R > SETPOINT)
            {
                if (speed1 > 0 && speed2 > 0)
                {
                    motor1->run_motor(-10);
                    motor2->run_motor(-10);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    motor1->run_motor(-50);
                    motor2->run_motor(-50);
                    delay(200);
                    pwm_stop(ARM_SERVO);
                    away();
                    return false;
                }
                else
                {
                    motor1->run_motor(10);
                    motor2->run_motor(10);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    motor1->run_motor(50);
                    motor2->run_motor(50);
                    delay(200);
                    pwm_stop(ARM_SERVO);
                    away();
                    return false;
                }
            }
        }
    }
    motor2->run_motor(0);
    motor1->run_motor(0);
    return true;
}

/**
 * Run a single motor at the given speed for the given number of 
 * milliseconds
 * @param motor: motor object 
 * @param speed: speed in [-100, 100]
 * @param ms: number of milliseconds the motor will run for
*/
void run1_for_ms(Motor *motor, short speed, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor->run_motor(speed);
    while (HAL_GetTick() - current_tick < ms)
    {
        short L = analogRead(L_SENSOR);
        short R = analogRead(R_SENSOR);

        if (L > SETPOINT || R > SETPOINT)
        {
            return;
        }
    }
    motor->run_motor(0);
    return;
}

void away()
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 800;
    while (HAL_GetTick() - t < turns)
    {
        left_motor.run_motor(-55);
        right_motor.run_motor(35);
    }
    run2_for_ms(&left_motor, &right_motor, 40, 40, 1000);
}

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
    }
    delay(1000);

    bool mot_1 = run2_for_ms(&right_motor, &left_motor, 100, 100, 500);
    if (!mot_1)
    {
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_2 = run2_for_ms(&right_motor, &left_motor, 50, 50, 300);
    if (!mot_2)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_3 = run2_for_ms(&right_motor, &left_motor, 50, 50, 200);
    if (!mot_3)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    bool mot_4 = run2_for_ms(&left_motor, &right_motor, -80, -80, 200);
    if (!mot_4)
    {
        return;
    }
    run2_for_ms(&left_motor, &right_motor, -100, -100, 300);
}

/**
 * Dumps the cans stored in the robot bin. Won't work if the battery isn't 
 * full
*/
void dump()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run2_for_ms(&left_motor, &right_motor, 0, 0, 10);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}