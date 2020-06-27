#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <NewPing.h>
#include "main.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Both TRIG and ECHO are Digital Pins
#define TRIG PA15
#define ECHO PB3
#define MAX_DISTANCE 300

Servo servo;
Motor left_motor = Motor(L_FORWARD, L_REVERSE);
Motor right_motor = Motor(R_FORWARD, R_REVERSE);
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

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

void loop()
{
    int current_tick = HAL_GetTick();

    if (current_tick < TOTAL_TIME - HOMING_TIME)
    {
        // unsigned long cm = sonar.ping_cm();

        for (int i = 0; i <= 100; i += 10){
            left_motor.run_motor(i);
            delay(200); 
            left_motor.run_motor(-1*i);
            delay(200); 
        }
        
        // int pg = analogRead(PGAIN) / 10;
        // int dg = analogRead(DGAIN);
        // // int dg = 0;
        // int intensity = analogRead(REFLECTANCE);

        // int error = (intensity - SETPOINT);

        // int p = pg * error;
        // int d = dg * (error - last_error);

        // int speed = p + d;
        // if (speed > 1023)
        // {
        //     speed = 1023;
        // }
        // else if (speed < -1023)
        // {
        //     speed = -1023;
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

        // linearization code
        // if ((speed<500)&&(speed>=0)) speed += 100;
        // if ((speed>-300)&&(speed<0)) speed -= 100;

        // last_error = error;
    }
    else
    {
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
};