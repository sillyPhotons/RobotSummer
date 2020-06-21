#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
// #include <NewPing.h> 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LED_DISPLAY PB12
#define TRIG PA15
#define ECHO PB3
#define MAX_DISTANCE 300 
// NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

void setup()
{
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    delay(500);
    display.setCursor(0, 0);
    display.print("Welcome, Ray.");
    display.display();
    delay(500);

    // attachInterrupt(digitalPinToInterrupt(LED_DISPLAY),
    //                 turn_on_display,
    //                 RISING);

    // servo.attach(SERVO1);
}

void loop()
{
    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    // // Read the signal from the sensor: a HIGH pulse whose
    // // duration is the time (in microseconds) from the sending
    // // of the ping to the reception of its echo off of an object.
    
    int duration = pulseIn(ECHO, HIGH);

    // // Convert the time into a distance
    double cm = (duration/2)*0.0343;     // Divide by 29.1 or multiply by 0.0343
    
    display.println(cm);
    display.display();
    display.clearDisplay();
    display.setCursor(0, 0);
    delay(250);
};
