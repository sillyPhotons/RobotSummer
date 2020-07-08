#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <NewPing.h>
#include "main.h"
#include <vector>
#include <numeric>

#define LED_DISPLAY PB12
// #define PGAIN PA5
// #define DGAIN PA4
// #define REFLECTANCE PB1
#define LED_DISPLAY PB12
#define BIN_SERVO PA8
#define ARM_SERVO PA6
#define IR PA7

// Both TRIG and ECHO are Digital Pins
#define TRIG PA11
#define ECHO PA12
#define MAX_DISTANCE 50

// Servo bin_servo;
Servo arm_servo;

// Left motor
#define L_FORWARD PB_8
#define L_REVERSE PB_9

// Right motor
#define R_FORWARD PA_1
#define R_REVERSE PA_0

Motor left_motor = Motor(L_FORWARD, L_REVERSE);
Motor right_motor = Motor(R_FORWARD, R_REVERSE);
Phototransistor ir_detection = Phototransistor(IR, 1000);
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void run2_for_ms(Motor motor1, Motor motor2, int speed1, int speed2, int ms);
void run1_for_ms(Motor motor1, int speed, int ms);
float detect_1KHz(int num_samples);
bool search(int max_distance, int spin_time, int increments);
void zero_in();

void zero_in()
{
}
/*
    @param max_distance: maximum distance in centimeters
    @param spin_time: time in ms
*/
bool search(int max_distance, int spin_time, int increments)
{
    // std::vector<int> history(spin_time);

    int closest = max_distance;
    int start_tick = HAL_GetTick();

    int ms_per_increment = spin_time / increments;
    while (HAL_GetTick() - start_tick < spin_time)
    {
        run2_for_ms(left_motor, right_motor, 40, -40, ms_per_increment);
        unsigned long cm = sonar.ping_cm();
        delay(500);
        if (cm < closest)
        {
            closest = cm;
        }
    }

    if (closest == max_distance)
    {
        return false;
    }

    unsigned long scan = sonar.ping_cm();
    while (HAL_GetTick() - start_tick < spin_time * 3)
    {
        run2_for_ms(left_motor, right_motor, 40, -40, ms_per_increment);
        unsigned long cm = sonar.ping_cm();
        delay(500);
        double difference = 1.0 * cm - 1.0 * closest;
        if (abs(difference) / (1.0 * cm + 1.0 * closest) / 2.0 < 0.1)
        {
            return true;
        }
    }
    return false;
}

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

float detect_1KHz(int num_samples)
{
    float raw[num_samples] = {0};

    int start = HAL_GetTick();
    for (int i = 0; i < num_samples; i += 1)
    {   
        raw[i] = analogRead(PA7);
        delay(1);
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
    double frequency_intensity = sqrt((real * real) + (imaginary * imaginary));

    return frequency_intensity;
}

void setup()
{
    // Set up display
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    delay(1000);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // attachInterrupt(digitalPinToInterrupt(LED_DISPLAY),
    //                 turn_on_display,
    //                 RISING);

    // bin_servo.attach(BIN_SERVO);
    arm_servo.attach(ARM_SERVO);
}

int test_cm = 30;
int max_distance = MAX_DISTANCE;
void loop()
{
    // run2_for_ms(right_motor, left_motor, 100, 100, 1000);
    // pwm_start(PA_6, 50, 2000, MICROSEC_COMPARE_FORMAT);
    // run2_for_ms(right_motor, left_motor, 50, 50, 500);
    // pwm_start(PA_6, 50, 1250, MICROSEC_COMPARE_FORMAT);
    // delay(5000);
    // pwm_start(PA_6, 50, 2550, MICROSEC_COMPARE_FORMAT);
    // delay(10000);

    float intensity = detect_1KHz(100);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(intensity);
    display.display();

    // bool found = search(max_distance, 3000, 20);
    // if (found)
    // {
    //     run2_for_ms(right_motor, left_motor, 100, 100, 500);
    // }
    // else
    // {
    //     max_distance += 10;
    // }

    // int current_tick = HAL_GetTick();

    // if (current_tick < TOTAL_TIME - HOMING_TIME)
    // {

    // left_motor.run_motor(50);

    // if (cm > test_cm)
    // {
    // }
    // else if (cm < test_cm)
    // {
    //     right_motor.run_motor(-20);
    // }
    // else
    // {
    //     right_motor.run_motor(0);
    // }
    // }
    // int pg = analogRead(PGAIN) / 10;
    // int dg = analogRead(DGAIN);
    // // int dg = 0;
    // int intensity = analogRead(REFLECTANCE);

    // int error = (intensity - SETPOINT);

    // int p = pg * error;
    // int d = dg * (error - last_error);

    // int speed = p + d;
    // if (speed > 100)
    // {
    //     speed = 100;
    // }
    // else if (speed < -100)
    // {
    //     speed = -100;
    // }

    // if (digitalRead(LED_DISPLAY) == LOW)
    // {
    //     run_motor(0);
    //     print_to_display(intensity, pg, dg, error, speed, cm);
    // }

    // else
    // {
    //     run_motor(speed);
    // }

    // last_error = error;
    // }
    // else
    // {
    //     left_motor.run_motor(0);
    //     right_motor.run_motor(0);
    // }
};