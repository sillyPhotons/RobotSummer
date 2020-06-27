/**
 * Header File for Main
 **/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

#define LED_DISPLAY PB12

#define PGAIN PA5
#define DGAIN PA4
#define REFLECTANCE PB1
#define LED_DISPLAY PB12
#define SERVO1 PB0

// Left motor
#define L_FORWARD PB_8
#define L_REVERSE PB_9

// Right motor
#define R_FORWARD PA_1
#define R_REVERSE PA_0

// Timing variables
const int TOTAL_TIME = 60000;
int HOMING_TIME = 10000;

// PID Control variables
const int SETPOINT = 250;
const int GAIN = 5;
int last_error = 0;

void print_to_display(int intensity, int pg, int dg, int error, int speed, double dist);

class Motor
{
public:
    Motor(PinName forward_pin, PinName reverse_pin) : forward_pin(forward_pin), reverse_pin(reverse_pin){};
    /*
        speed in [-1023, 1023]
        @params speed
    */
    void run_motor(int speed)
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

private:
    PinName forward_pin = PC_13;
    PinName reverse_pin = PC_13;
};
