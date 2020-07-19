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

// Timing variables
const int start_time = HAL_GetTick();
const int TAPE_TIME = 3000;
const int TOTAL_TIME = 60000;
const int HOMING_TIME = 20000;
const int BACK_UP_TIME = 1000;

// PID Control variables

// line following
short SETPOINT = 300;
const short GAIN = 5;
short prevError = 0;
const short LINE_FOLLOW_SPEED = 25;
short max_adjustment_speed = LINE_FOLLOW_SPEED + 10;
float P, D, error = 0;
short Lspeed = LINE_FOLLOW_SPEED;
short Rspeed = LINE_FOLLOW_SPEED;

bool dumped = false;

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
