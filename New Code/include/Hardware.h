#include <main.h>
#include <Adafruit_SSD1306.h>

#ifndef HARDWARE_H
#define HARDWARE_H

#define PGAIN PA5 // Potentiometer
#define DGAIN PA4 // Potentiometer

#define LED_DISPLAY PB12 // OLED

// Left motor
#define MOTOR_LF PB_8
#define MOTOR_LB PB_9
// Right motor
#define MOTOR_RF PA_1
#define MOTOR_RB PA_0
#define R_SENSOR PB1 // Reflectance Sensor
#define L_SENSOR PB0 // Reflectance Sensor
#define IR PA7       // Phototransistor

#define ARM_SERVO PA_6
#define BIN_SERVO PA_8

#define ARM_REST 2550
#define ARM_H_UP1 2200
#define ARM_H_UP2 1800
#define ARM_UP 1165
#define BIN_REST 820
#define BIN_UP 1900

#define TRIG PA11  // Sonar 1 Trigger
#define ECHO PA12  // Sonar 2 Echo
#define TRIG2 PB12 // Sonar 2 Trigger
#define ECHO2 PB13 // Sonar 2 Echo

class Motor
{
public:
    Motor(PinName forward_pin, PinName reverse_pin) : forward_pin(forward_pin), reverse_pin(reverse_pin){};
    // speed in [-100, 100]
    void run_motor(short speed);

    /**
     * Returns last nonzero speed
     */
    short get_last_speed()
    {
        return current_speed;
    }

private:
    PinName forward_pin = PC_13;
    PinName reverse_pin = PC_13;
    short current_speed = 0;
};

static NewPing sonar(TRIG, ECHO, 200);
static NewPing sonar2(TRIG2, ECHO2, 200);
static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static Motor left_motor = Motor(MOTOR_LF, MOTOR_LB);
static Motor right_motor = Motor(MOTOR_RF, MOTOR_RB);

static HardwareTimer MyTim = HardwareTimer(TIM1);


extern short L_reading;
extern short R_reading;
extern volatile bool crossed;
extern volatile bool L_crossed;
extern volatile bool R_crossed;

void dump();
void away();
void crossed_tape();
void check_reflectance();
void check_crossed_tape();
void pick_up_can(bool correction);
bool run_both(short left_speed, short right_speed, unsigned int ms, bool trigger_away);

#endif