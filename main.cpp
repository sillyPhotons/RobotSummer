#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <NewPing.h>
#include "main.h"
#include <vector>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Both TRIG and ECHO are Digital Pins
#define TRIG PA11
#define ECHO PA12
#define MAX_DISTANCE 300

Servo servo;
Motor left_motor = Motor(L_FORWARD, L_REVERSE);
Motor right_motor = Motor(R_FORWARD, R_REVERSE);
Phototransistor ir_detection = Phototransistor(IR);
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

void print_to_display(int intensity, int pg, int dg, int error, int speed, double dist);

void run2_for_ms(Motor motor1, Motor motor2, int speed1, int speed2, int ms);
void run1_for_ms(Motor motor1, int speed, int ms);
void search(void);

void setup()
{

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    delay(500);
    display.setCursor(0, 0);
    display.print("Welcome, Ray.");
    display.display();
    delay(500);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(LED_DISPLAY),
    //                 turn_on_display,
    //                 RISING);

    // servo.attach(SERVO1);
}

void search(int increments, int ms_per_increment)
{
    int g_i = 0;
    int g = 100000;
    std::vector<int> history(increments);

    for (int i = 0; i < increments / 2; i += 1)
    {
        run2_for_ms(right_motor, left_motor, 50, -50, ms_per_increment);
        unsigned long cm = sonar.ping_cm();
        delay(100);
        history[i] = cm;
        if (cm < g)
        {
            g = cm;
            g_i = i;
        }
    }
    for (int i = 0; i < increments / 2; i += 1)
    {
        run2_for_ms(right_motor, left_motor, -50, 50, ms_per_increment);
        delay(100);
    }
    // run1_for_ms(right_motor, -50, ms_per_increment* increments/2);
    for (int i = increments / 2 - 1; i < increments; i += 1)
    {
        run2_for_ms(left_motor, right_motor, 50, -50, ms_per_increment);
        unsigned long cm = sonar.ping_cm();
        delay(100);
        history[i] = cm;
        if (cm < g)
        {
            g = cm;
            g_i = i;
        }
    }
    for (int i = increments / 2 - 1; i < increments; i += 1)
    {
        run2_for_ms(left_motor, right_motor, -50, 50, ms_per_increment);
        delay(100);
    }
    // run1_for_ms(left_motor, -50, ms_per_increment * increments/2);

    if (g_i < increments / 2)
    {   
        for (int i = 0; i < increments / 2; i += 1)
        {
            run2_for_ms(right_motor, left_motor, 50, -50, ms_per_increment);
            unsigned long cm = sonar.ping_cm();
            delay(100);
            int difference = g - cm;
            if (abs(difference) < 5)
            {   
                run2_for_ms(left_motor, right_motor, 70, 70, 1000);
                return;
            }
        }
    }
    
    if (g_i > increments / 2)
    {   
        for (int i = increments / 2 - 1; i < increments; i += 1)
        {   
            run2_for_ms(left_motor, right_motor, 50, -50, ms_per_increment);
            unsigned long cm = sonar.ping_cm();
            delay(100);   
            int difference = g - cm;
            if (abs(difference) < 5)
            {   
                run2_for_ms(left_motor, right_motor, 70, 70, 1000);
                return;
            }
        }
    }
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
}

void run1_for_ms(Motor motor, int speed, int ms)
{
    int current_tick = HAL_GetTick();
    while (HAL_GetTick() - current_tick < ms)
    {
        motor.run_motor(speed);
    }
    motor.run_motor(0);
}
// void print_to_display(int intensity, int pg, int dg, int error, int speed, double dist)
// {
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.print("Sonar: ");
//     display.println(dist);
//     display.print("Intensity: ");
//     display.println(intensity);
//     display.display();
//     display.print("PGain: ");
//     display.println(pg);
//     display.print("DGain: ");
//     display.println(dg);
//     display.print("Error: ");
//     display.println(error);
//     display.print("Speed: ");
//     display.println(speed);
//     display.display();
//     delay(100);
// }

int test_cm = 30;

void loop()
{
    // int intensity = analogRead(ir_detection.get_pin());
    // display.clearDisplay();
    // display.setCursor(0, 0);
    // display.print(intensity);
    // delay(100);
    search(20, 50);
    return; 

    int current_tick = HAL_GetTick();

    if (current_tick < TOTAL_TIME - HOMING_TIME)
    {   
        
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
    }
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
    else
    {
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
};