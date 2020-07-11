#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include "main.h"
#include <vector>
#include <numeric>

#define LED_DISPLAY PB12

#define PGAIN PA5
#define DGAIN PA4
#define R_SENSOR PB1
#define L_SENSOR PB0

#define BIN_REST 820
#define BIN_UP 1900

#define BIN_SERVO PA_8
#define ARM_REST 2550
#define ARM_H_UP 2000
#define ARM_UP 1250
#define ARM_SERVO PA_6

#define IR PA7

#define LEFT_RF PB1
#define RIGHT_RF PB0

// Both TRIG and ECHO are Digital Pins
#define TRIG PA11
#define ECHO PA12
#define MAX_DISTANCE 50
#define TARGET_DISTANCE 30

// Left motor
#define MOTOR_LF PB_8
#define MOTOR_LB PB_9

// Right motor
#define MOTOR_RF PA_1
#define MOTOR_RB PA_0

Motor left_motor = Motor(MOTOR_LF, MOTOR_LB);
Motor right_motor = Motor(MOTOR_RF, MOTOR_RB);
NewPing sonar(TRIG, ECHO, 200);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void run2_for_ms(Motor motor1, Motor motor2, int speed1, int speed2, int ms);
void run1_for_ms(Motor motor1, int speed, int ms);
float detect_1KHz(int num_samples);
bool search(int max_distance, int spin_time, int increments);
void PID(float L, float R);
void checkLine();
void pick_up_can();
void dump();
void setup()
{
    delay(100);
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    delay(500);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(R_SENSOR, INPUT);
    pinMode(L_SENSOR, INPUT);
    pinMode(MOTOR_RF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_LB, OUTPUT);

    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
}

/*
    Searches within the search radius for any objects
    @param search_radius: centimeters 
*/
bool search(int search_radius)
{
    int t = HAL_GetTick();
    float preError = 0;
    bool found = false;

    // This function will last for 4 seconds
    while (HAL_GetTick() - t < 3500)
    {
        int cm = sonar.ping_cm(); // take distance measurement
        int error = cm - TARGET_DISTANCE;

        // object is within search radius
        // error will equal -1*TARGET_DISTANCE if the object is out of range
        if (cm < search_radius && error != -1.0 * TARGET_DISTANCE)
        {
            found = true;

            if (error > 0)
            {
                float P = 0 * error;
                float D = 0 * (error - preError);
                prevError = error;
                float adj = P + D;

                Lspeed = constrain(adj, 20, 40);
                Rspeed = constrain(adj, 20, 40);

                left_motor.run_motor(20 + adj);
                right_motor.run_motor(20 + adj);
                display.clearDisplay();
                display.setCursor(0, 0);
                display.print("Moving Forward: ");
                display.print(adj);
                display.display();
            }

            else if (error < 0)
            {
                float P = 1.5 * error;
                float D = 0 * (error - preError);
                prevError = error;
                float adj = P + D;

                display.clearDisplay();
                display.setCursor(0, 0);
                display.print("ERROR DETECTED: ");
                display.println(error);

                /* 
                    The following depends on you. My left motor is always 
                    slower than the right when going in reverse. So here I try 
                    to apply various corrections 
                */
                Lspeed = constrain(adj, -1 * 50, 40);
                Rspeed = constrain(adj, -1 * 40, 40);

                if (Lspeed > -1 * 40 && Lspeed < 0)
                {
                    Lspeed = map(Lspeed, -50, 0, -50, -42);
                }
                left_motor.run_motor(Lspeed);
                right_motor.run_motor(Rspeed);
                display.println(Lspeed);
                display.println(Rspeed);
                display.display();
            }
        }
        else
        {
            found = false;
            run2_for_ms(left_motor, right_motor, -55, 35, 50);
            run2_for_ms(left_motor, right_motor, 0, 0, 150);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("SEARCH with radius: ");
            display.println(search_radius);
            display.display();
        }
    }
    return found;
}

/*
    Run a two motor at the given speed for the given number of milliseconds

    @param motor1, motor2: motor objects
    @param speed1, speed2: speed in [-100, 100]
    @param ms: number of milliseconds the motors will run for
*/
void run2_for_ms(Motor motor1, Motor motor2, int speed1, int speed2, int ms)
{
    int current_tick = HAL_GetTick();
    while (HAL_GetTick() - current_tick < ms)
    {
        motor1.run_motor(speed1);
        motor2.run_motor(speed2);
    }
    motor2.run_motor(0);
    motor1.run_motor(0);
    return;
}

/*
    Run a single motor at the given speed for the given number of milliseconds

    @param motor: motor object
    @param speed: speed in [-100, 100]
    @param ms: number of milliseconds the motor will run for
*/
void run1_for_ms(Motor motor, int speed, int ms)
{
    int current_tick = HAL_GetTick();
    while (HAL_GetTick() - current_tick < ms)
    {
        motor.run_motor(speed);
    }
    motor.run_motor(0);
    return;
}

/*
    Returns the signal frequency intensity of thes detected IR signal.
    @param num_samples: number of samples to take. 
*/
float detect_1KHz(int num_samples)
{
    float raw[num_samples] = {0};

    int start = HAL_GetTick();
    for (int i = 0; i < num_samples; i += 1)
    {
        raw[i] = analogRead(PA7);
        delay(1); // single sample every ~ 1 ms
    }
    int ms_taken = HAL_GetTick() - start;
    double sampling_period = ms_taken / (num_samples * 1000.0);
    int sampling_hz = num_samples / sampling_period; // in Hz

    float ref_freq = 1000.0;

    float cosine_ref[num_samples] = {0};
    float sine_ref[num_samples] = {0};

    for (int i = 0; i < num_samples; i += 1)
    {
        cosine_ref[i] = cos(i * 2 * PI * ref_freq / sampling_hz);
        sine_ref[i] = sin(i * 2 * PI * ref_freq / sampling_hz);
    }

    float real = std::inner_product(raw, raw + sizeof(raw) / sizeof(raw[0]), cosine_ref, 0);
    float imaginary = std::inner_product(raw, raw + sizeof(raw) / sizeof(raw[0]), sine_ref, 0);
    float frequency_intensity = sqrt((real * real) + (imaginary * imaginary));

    return frequency_intensity;
}

/*
    TODO: Finish this function!!
*/
void to_IR_BEACON(int threshold)
{

    int i = 1;
    bool found_peak = false;
    bool converged = false;
    float prev_max = 0;

    while (!found_peak)
    {
        run2_for_ms(left_motor, right_motor, i * 20, -1 * i * 20, 200);
        float history = 0;
        for (int i = 0; i < 10; i += 1)
        {
            history += detect_1KHz(50);
        }
        float average = history / 10;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print(average);
        display.display();

        if (average < threshold)
        {
        }
        else
        {
            found_peak = true;
            prev_max = average;
        }
    }

    while (!converged)
    {
        run2_for_ms(left_motor, right_motor, -49, -30, 200);

        float history = 0;
        for (int i = 0; i < 20; i += 1)
        {
            history += detect_1KHz(50);
        }
        float average = history / 10;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print(average);
        display.display();

        if (abs(average - prev_max) / ((average + prev_max) / 2.0) < 0.2 || average > prev_max)
        {
        }
        else
        {
            return;
        }
    }
}

/*
Checks if robot sees the line or not.
If yes, PID line following.
If no, sharp turn based on previous location to find the line
*/
void checkLine()
{
    float L = analogRead(L_SENSOR);
    float R = analogRead(R_SENSOR);

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
    else
    {
        PID(L, R);
    }
}

/*
Computes the adjustment for the motors based on how far off the sensors
 are from the line and changes motor speed.

 @param L analog reading from left TCRT5000
 @param R analog reading from right TCRT5000
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

void pick_up_can()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(1000);
    run2_for_ms(right_motor, left_motor, 100, 100, 500);
    pwm_start(ARM_SERVO, 50, ARM_H_UP, MICROSEC_COMPARE_FORMAT);
    run2_for_ms(right_motor, left_motor, 50, 50, 300);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    run2_for_ms(right_motor, left_motor, 50, 50, 300);
}

/*
    Dumps the cans stored in the robot bin. Won't work if the battery isn't full
*/
void dump()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run2_for_ms(left_motor, right_motor, 0, 0, 10);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    dumped = true;
}

/*
    Only Cans within max_distance will be detected
    - May change with every loop iteration
*/
int max_distance = MAX_DISTANCE;

/*
    Event loop
*/
void loop()
{

    // Get time since start in ms
    int time_elapsed = HAL_GetTick() - start_time;
    // Fixed time to travel on tape
    if (time_elapsed < TAPE_TIME)
    {
        checkLine(); // line following
    }
    // After tape start search
    else if (time_elapsed < TOTAL_TIME - HOMING_TIME)
    {
        bool found = search(max_distance);
        if (!found)
        {
            max_distance += 15; // increase search radius if nothing found
        }
        if (found)
        {
            max_distance = MAX_DISTANCE;
            // alignment correction
            run2_for_ms(left_motor, right_motor, -55, 35, 50);
            pick_up_can();
            // Reverse the robot after raming into the can
            run2_for_ms(left_motor, right_motor, -90, -90, 200);
            run2_for_ms(left_motor, right_motor, -100, -100, 500);
            delay(1000);
        }
    }
    else if (time_elapsed < TOTAL_TIME && !dumped)
    {
        // Requires high current
        dump();
    }

    // to_IR_BEACON(100);

    // run2_for_ms(right_motor, left_motor, 100, 100, 1000);
    // pwm_start(ARM_SERVO, 50, 2000, MICROSEC_COMPARE_FORMAT);
    // run2_for_ms(right_motor, left_motor, 50, 50, 500);
    // pwm_start(ARM_SERVO, 50, 1550, MICROSEC_COMPARE_FORMAT);
    // run2_for_ms(right_motor, left_motor, 50, 50, 500);
    // pwm_start(ARM_SERVO, 50, 1250, MICROSEC_COMPARE_FORMAT);
    // delay(5000);
    // pwm_start(ARM_SERVO, 50, 2550, MICROSEC_COMPARE_FORMAT);
    // delay(5000);
    // pwm_start(BIN_SERVO, 50, 830, MICROSEC_COMPARE_FORMAT);
    // delay(5000);
    // pwm_start(BIN_SERVO, 50, 1900, MICROSEC_COMPARE_FORMAT);
    // delay(5000);

    // float lrf = analogRead(LEFT_RF);
    // float rrf = analogRead(RIGHT_RF);
    // display.clearDisplay();
    // display.setCursor(0, 0);
    // display.println(lrf);
    // display.println(rrf);
    // display.display();
};