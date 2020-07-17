#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include <main.h>
#include <vector>
#include <numeric>
#include <Bitmap.h>

// #include "stm32f10x.h"
// #include <stm32f1xx_hal_adc.h>
// #include <stm32f1xx_hal.h>
// #include <stm32f1xx_hal_gpio.h>
// #include <stm32f1xx.h>

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
#define MAX_DISTANCE 60
#define TARGET_DISTANCE 30

#define TRIG2 PB12
#define ECHO2 PB13

// Left motor
#define MOTOR_LF PB_8
#define MOTOR_LB PB_9

// Right motor
#define MOTOR_RF PA_1
#define MOTOR_RB PA_0

#define ADC1_DR_Address ((uint32_t)0x4001244C)

Motor left_motor = Motor(MOTOR_LF, MOTOR_LB);
Motor right_motor = Motor(MOTOR_RF, MOTOR_RB);
NewPing sonar(TRIG, ECHO, 200);
NewPing sonar2(TRIG2, ECHO2, 200);

Adafruit_SSD1306 display(128, 64, &Wire, -1);

bool run2_for_ms(Motor *motor1, Motor *motor2, int speed1, int speed2, unsigned int ms);
void run1_for_ms(Motor *motor1, int speed, unsigned int ms);
float detect_1KHz(int num_samples);
bool search(int max_distance, int spin_time, int increments);
void PID(float L, float R);
bool checkLine();
void pick_up_can(bool correction);
void dump();
bool align();
void new_direction(int travel_time);
// void half_turn();
// void ConfigureADC();

ADC_HandleTypeDef g_AdcHandle;
/*
    Only Cans within max_distance will be detected
    - May change with every loop iteration
*/
int max_distance = MAX_DISTANCE;
int l_or_r = 1;
bool line_following = false;

void setup()
{
    Serial1.begin(115200);

    delay(100);
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.drawBitmap(0, 0, fizzBitMap, 128, 64, WHITE);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    delay(200);
    display.clearDisplay();
    display.setCursor(0, 0);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(R_SENSOR, INPUT);
    pinMode(L_SENSOR, INPUT);
    pinMode(MOTOR_RF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_LB, OUTPUT);

    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
    // HAL_Init();
    // ConfigureADC();
}

void ConfigureADC()
{

    g_AdcHandle.Instance = ADC1;
    g_AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    g_AdcHandle.Init.ContinuousConvMode = ENABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&g_AdcHandle);

    ADC_ChannelConfTypeDef IRConfig;
    IRConfig.Channel = ADC_CHANNEL_7;
    IRConfig.Rank = ADC_REGULAR_RANK_1;
    IRConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    // ADC_ChannelConfTypeDef RConfig;
    // IRConfig.Channel = ADC_CHANNEL_9;
    // IRConfig.Rank = ADC_REGULAR_RANK_2;
    // IRConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    // ADC_ChannelConfTypeDef LConfig;
    // IRConfig.Channel = ADC_CHANNEL_8;
    // IRConfig.Rank = ADC_REGULAR_RANK_2;
    // IRConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    HAL_ADC_ConfigChannel(&g_AdcHandle, &IRConfig);
}

bool found = false;
const int search_time = 7000;
const int align_time = 1500;

bool align()
{
    int t = HAL_GetTick();
    int preError = 0;

    // aligning
    while (HAL_GetTick() - t < align_time)
    {
        int cm = sonar.ping_cm(); // take distance measurement
        int error = cm - TARGET_DISTANCE;

        // move forward
        if (error > 0)
        {
            // float P = 0 * error;
            // float D = 0 * (error - preError);
            // float adj = P + D;

            // Lspeed = constrain(adj, 20, 40);
            // Rspeed = constrain(adj, 20, 40);
            // left_motor.run_motor(20 + adj);
            // right_motor.run_motor(20 + adj);

            left_motor.run_motor(20);
            right_motor.run_motor(20);
            // display.clearDisplay();
            // display.setCursor(0, 0);
            // display.print("Moving Forward: ");
            // display.print(adj);
            // display.display();
        }

        else if (error < 0)
        {
            float P = 2.0 * error;
            float D = 0 * (error - preError);
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
            Rspeed = constrain(adj, -1 * 50, 40);

            if (Lspeed > -1 * 40 && Lspeed < 0)
            {
                Lspeed = map(Lspeed, -50, 0, -47, -30);
            }
            left_motor.run_motor(Lspeed);
            right_motor.run_motor(Rspeed);
            display.println(Lspeed);
            display.println(Rspeed);
            display.display();
        }

        prevError = error;
    }

    int cm = sonar.ping_cm(); // take distance measurement
    int error = cm - TARGET_DISTANCE;
    bool complete = false;
    if (abs(error) < 3)
    {
        complete = true;
    }
    return complete;
}

bool linear_search(int search_radius, int travel_time)
{

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(travel_time / 1000);
    display.print("s L-SEARCH");
    display.display();

    int t = HAL_GetTick();

    // searching
    while (HAL_GetTick() - t < travel_time)
    {
        int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
        int cm = sonar.ping_cm();   // take distance measurement

        // if cm2 is 0, then that means object is outside of detection range
        if (cm2 == 0)
        {
            cm2 = 200;
        }
        if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 40)
        {
            Serial1.println("FOUND!");
            found = true;
            break;
        }
        else
        {
            run2_for_ms(&left_motor, &right_motor, 40, 40, 100);
            run2_for_ms(&left_motor, &right_motor, 0, 0, 100);
        }
    }
    return found;
}

/**
  * Searches within the search radius for any objects
  * @param search_radius: centimeters
  * @param l_or_r: 1 is right turn search, -1 is left turn search
  */
bool search(int search_radius, int l_or_r)
{
    int t = HAL_GetTick();

    // searching
    while (HAL_GetTick() - t < search_time)
    {
        int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
        int cm = sonar.ping_cm();   // take distance measurement

        // if cm2 is 0, then that means object is outside of detection range
        if (cm2 == 0)
        {
            cm2 = 200;
        }

        // object is within search radius
        // error will equal -1*TARGET_DISTANCE if the object is out of range
        // the top sensor detects at least twice the distance
        // cm2 has minimum 40 **THIS IS CAUSE MY SENSOR WILL NEVER RETURN VALUE LESS THAN 20**!!
        if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 40)
        {
            Serial1.println("FOUND!");
            found = true;
            break;
        }
        else
        {
            found = false;
            if (l_or_r == 1)
            {
                run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 150);

                Serial1.print("RIGHT with radius: ");
                Serial1.println(search_radius);
            }
            else
            {
                run2_for_ms(&left_motor, &right_motor, 35, -35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 150);

                Serial1.print("LEFT with radius: ");
                Serial1.println(search_radius);
            }
            // display.clearDisplay();
            // display.setCursor(0, 0);
            // display.print("SEARCH with radius: ");
            // display.println(search_radius);
            // display.display();
        }
    }
    return found;
}

/*
    Run a two motor at the given speed for the given number of milliseconds

    @param motor1, motor2: motor objects
    @param speed1, speed2: speed in [-100, 100]
    @param ms: number of milliseconds the motors will run for

    @returns: a boolean. True if the action was completed, false if it was interrupted
*/
bool run2_for_ms(Motor *motor1, Motor *motor2, int speed1, int speed2, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor1->run_motor(speed1);
    motor2->run_motor(speed2);
    while (HAL_GetTick() - current_tick < ms)
    {
        if ((speed1 > 0 && speed2 > 0) || (speed1 < 0 && speed2 < 0))
        {
            float L = analogRead(L_SENSOR);
            float R = analogRead(R_SENSOR);

            if (L > SETPOINT || R > SETPOINT)
            {
                motor1->run_motor(-10);
                motor2->run_motor(-10);
                pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                delay(500);
                motor1->run_motor(-50);
                motor2->run_motor(-50);
                delay(200);
                pwm_stop(ARM_SERVO);
                new_direction(1000);
                return false;
            }
        }
    }
    motor2->run_motor(0);
    motor1->run_motor(0);
    return true;
}

/*
    Run a single motor at the given speed for the given number of milliseconds

    @param motor: motor object
    @param speed: speed in [-100, 100]
    @param ms: number of milliseconds the motor will run for
*/
void run1_for_ms(Motor *motor, int speed, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor->run_motor(speed);
    while (HAL_GetTick() - current_tick < ms)
    {
        float L = analogRead(L_SENSOR);
        float R = analogRead(R_SENSOR);

        if (L > SETPOINT || R > SETPOINT)
        {
            return;
        }
        // if (speed > 0)
        // {
        //     int cm2 = sonar2.ping_cm();
        //     if (cm2 <= TARGET_DISTANCE && cm2 != 0)
        //     {
        //         break;
        //     }
        // }
    }
    motor->run_motor(0);
    return;
}

void new_direction(int travel_time)
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 1000;
    while (HAL_GetTick() - t < turns)
    {
        left_motor.run_motor(-55);
        right_motor.run_motor(35);
    }
    linear_search(50, travel_time);
}

void away_from_boundary()
{
    // left_motor.run_motor(0);
    // right_motor.run_motor(0);
    // float L = analogRead(L_SENSOR);
    // float R = analogRead(R_SENSOR);
    // display.clearDisplay();
    // display.setCursor(0,0);
    // display.println(L);
    // display.println(R);
    // display.display();

    // delay(5000);

    right_motor.run_motor(LINE_FOLLOW_SPEED);
    left_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
    prevError = 1;
    unsigned int t = HAL_GetTick();
    while (HAL_GetTick() - t < 1000)
    {
        checkLine();
    }
    right_motor.run_motor(LINE_FOLLOW_SPEED);
    left_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
    delay(300);
}
/*
    Returns the signal frequency intensity of thes detected IR signal.
    @param num_samples: number of samples to take. 
*/
float detect_1KHz(int num_samples)
{
    float raw[num_samples] = {0};
    int start = HAL_GetTick();
    HAL_ADC_Start(&g_AdcHandle);
    for (int i = 0; i < num_samples; i += 1)
    {
        if (HAL_ADC_PollForConversion(&g_AdcHandle, 100) == HAL_OK)
        {
            raw[i] = 3.3 * HAL_ADC_GetValue(&g_AdcHandle) / 4096.0;
        }
    }
    HAL_ADC_Stop(&g_AdcHandle);
    int ms_taken = HAL_GetTick() - start;
    float sampling_period = ms_taken / (num_samples * 1000.0);
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
void to_IR_BEACON()
{
    float L = 0;
    float R = 0;
    // analogRead(L_SENSOR);
    // float R = analogRead(R_SENSOR);
    // Serial1.println(L);
    // Serial1.println(R);

    if (L < SETPOINT && R < SETPOINT)
    {
        float intensity = detect_1KHz(100);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(intensity);
        display.display();

        if (intensity != 100.0)
        {
            left_motor.run_motor(30);
            right_motor.run_motor(30);
        }
        else
        {
            if (l_or_r == 1)
            {
                Serial1.println("Not found, turning right");
                left_motor.run_motor(-40);
                right_motor.run_motor(20);
                intensity = detect_1KHz(100);
                if (intensity == 100.0)
                {
                    l_or_r = -1;
                    return;
                }
            }
            else
            {
                Serial1.println("Not found, turning left");
                left_motor.run_motor(20);
                right_motor.run_motor(-20);
                intensity = detect_1KHz(100);
                if (intensity == 100.0)
                {
                    l_or_r = 1;
                    return;
                }
            }
        }
    }
}

/*
Checks if robot sees the line or not.
If yes, PID line following.
If no, sharp turn based on previous location to find the line

returns: true if complete
*/
bool checkLine()
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
        if (line_following)
        {
            return true;
        }
    }
    else
    {
        PID(L, R);
        return false;
    }

    return false;
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

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
    }
    delay(1000);

    run2_for_ms(&right_motor, &left_motor, 100, 100, 500);
    pwm_start(ARM_SERVO, 50, ARM_H_UP, MICROSEC_COMPARE_FORMAT);
    run2_for_ms(&right_motor, &left_motor, 50, 50, 300);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    run2_for_ms(&right_motor, &left_motor, 50, 50, 200);
    run2_for_ms(&left_motor, &right_motor, -80, -80, 200);
    run2_for_ms(&left_motor, &right_motor, -100, -100, 300);
}

/*
    Dumps the cans stored in the robot bin. Won't work if the battery isn't full
*/
void dump()
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run2_for_ms(&left_motor, &right_motor, 0, 0, 10);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    dumped = true;
}

/*
    Event loop
*/
void loop()
{
    // run2_for_ms(&left_motor, &right_motor, 20, 20, TAPE_TIME);

    // run2_for_ms(&left_motor, &right_motor, 20, 20, 50);
    // // checkLine();
    // found = search(50, l_or_r);
    // if (found)
    // {
    //     int complete = align();
    //     if (!complete)
    //     {
    //         found = false;
    //         // l_or_r *= -1;
    //     }
    //     if (complete)
    //     {
    //         pick_up_can(true);
    //         delay(1000);
    //     }
    // }
    // else {
    //     new_direction(3000);
    // }

    // checkLine();

    // Get time since start in ms
    int time_elapsed = HAL_GetTick() - start_time;
    // Fixed time to travel on tape
    if (time_elapsed < TAPE_TIME)
    {
        left_motor.run_motor(20);
        right_motor.run_motor(20);
    }
    // // After tape start search
    else if (time_elapsed < TOTAL_TIME - HOMING_TIME)
    {
        found = search(50, l_or_r);
        if (!found)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("New Direction");
            display.display();
            new_direction(3000);
        }
        if (found)
        {
            int complete = align();
            if (!complete)
            {
                found = false;
                // l_or_r *= -1;
            }
            if (complete)
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.println("Aligned");
                display.display();
                pick_up_can(true);
                delay(1000);
            }
        }
    }
    else if (time_elapsed < TOTAL_TIME & !line_following)
    {
        float L = analogRead(L_SENSOR);
        float R = analogRead(R_SENSOR);

        if (L < SETPOINT || R < SETPOINT)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println(L);
            display.println(R);
            display.display();
            left_motor.run_motor(40);
            right_motor.run_motor(40);
        }

        else if (L >= SETPOINT || R >= SETPOINT)
        {
            while ((L >= SETPOINT && R >= SETPOINT) || (L < SETPOINT && R < SETPOINT))
            {
                L = analogRead(L_SENSOR);
                R = analogRead(R_SENSOR);
                run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 150);
            }
            line_following = true;
            checkLine();
        }
    }
    else if (time_elapsed < TOTAL_TIME && line_following)
    {
        bool end = checkLine();
        if (end)
        {
            unsigned int t = HAL_GetTick();
            while ((HAL_GetTick() - t) < BACK_UP_TIME)
            {
                left_motor.run_motor(-60);
                right_motor.run_motor(-60);
            }
        }
    }
    else if (!dumped)
    {
        dump();
    }

    // if (time_elapsed < TOTAL_TIME && !line_following)
    // {
    //     float L = analogRead(L_SENSOR);
    //     float R = analogRead(R_SENSOR);

    //     if (L < SETPOINT || R < SETPOINT)
    //     {
    //         display.clearDisplay();
    //         display.setCursor(0, 0);
    //         display.println(L);
    //         display.println(R);
    //         display.display();
    //         left_motor.run_motor(40);
    //         right_motor.run_motor(40);
    //     }

    //     else if (L >= SETPOINT || R >= SETPOINT)
    //     {
    //         while ((L >= SETPOINT && R >= SETPOINT) || (L < SETPOINT && R < SETPOINT))
    //         {
    //             L = analogRead(L_SENSOR);
    //             R = analogRead(R_SENSOR);
    //             run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
    //             run2_for_ms(&left_motor, &right_motor, 0, 0, 150);
    //         }
    //         line_following = true;
    //         checkLine();
    //     }
    // }
    // else if (time_elapsed < TOTAL_TIME && line_following)
    // {
    //     bool end = checkLine();
    //     if(end){
    //         unsigned int t = HAL_GetTick();
    //         while ((HAL_GetTick() - t) < BACK_UP_TIME)
    //         {
    //             left_motor.run_motor(-60);
    //             right_motor.run_motor(-60);
    //         }
    //     }
    // }
    // float intensity = detect_1KHz(100);
    // Serial1.println(intensity);
    // float L = analogRead(L_SENSOR);
    // float R = analogRead(R_SENSOR);
    // display.clearDisplay();
    // display.setCursor(0,0);
    // display.println(L);
    // display.println(R);
    // display.display();

    // Serial1.println(L);
    // Serial1.println(R);

    // delay(1000);
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