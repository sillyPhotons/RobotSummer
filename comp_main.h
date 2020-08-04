//Timing Constants
const int TOTAL_TIME = 60000; //total competition time
const int HOMING_TIME = 20000; //time to find way back
const int SEARCH_TIME = 7000; //time in one search iteration
const int START_TIME = 1300; //time for forward straight
const int LF_EXTRA_TIME = 5000;
const int ALIGN_TIME = 1000; //Time limit for aligning (ms)
const int NEW_DIRECTION_TRAVEL = 1100;
//OLED display setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

//potentiometer setup: used to set motor speed
#define POT PA5

//Motor setup
#define MOTOR_LF PB_8
#define MOTOR_LB PB_9
#define MOTOR_RF PA_0
#define MOTOR_RB PA_1
#define PWMFREQ 700 
#define LF_PWMFREQ 1200
//Sonar setup
#define L_TRIG PA11 //lower sonar
#define L_ECHO PA12 //lower sonar
#define U_TRIG PB12 //upper sonar
#define U_ECHO PB13 //upper sonar
#define MAX_DISTANCE 200

//Tape sensors
#define R_TAPE PB1
#define L_TAPE PB0
#define B_THRESHOLD 850
#define TF_THRESHOLD 300
#define E_THRESHOLD 900
#define INITIAL_SPEED 600.0

//PID constants
#define Ki 0.0
#define Kd 30
#define Kp 0.2

//Servo setup
#define ARM_SERVO PA_6
#define BUCKET PA_8
#define SWITCH PB5
#define BUCKET_DOWN 950
#define BUCKET_UP 2400

#define ARM_DOWN 950
#define ARM_H_UP 1700
#define ARM_UP 2240
#define ARM_UP_D 2000

//Other Constants
#define TARGET_DISTANCE 36 //how far away robot must be before picking up can (cm) 36 worked?
#define SEARCH_RADIUS 70 // How far the robot scans for cans (cm)
#define ERROR_LIM 6 // Error between detected distance and TARGET_DISTANCE (cm)


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
            speed = map(speed, 0, 100, 600, 1023);
            pwm_start(forward_pin, PWM_FREQUENCY, speed, RESOLUTION_10B_COMPARE_FORMAT);
            pwm_start(reverse_pin, PWM_FREQUENCY, 0, RESOLUTION_10B_COMPARE_FORMAT);
        }
        else if (speed < 0)
        {
            speed = map(speed, -100, 0, -1023, -800);
            pwm_start(reverse_pin, PWM_FREQUENCY * 5, speed * -1, RESOLUTION_10B_COMPARE_FORMAT);
            pwm_start(forward_pin, PWM_FREQUENCY * 5, 0, RESOLUTION_10B_COMPARE_FORMAT);
        }
        else
        {
            pwm_start(reverse_pin, PWM_FREQUENCY, 0, RESOLUTION_10B_COMPARE_FORMAT);
            pwm_start(forward_pin, PWM_FREQUENCY, 0, RESOLUTION_10B_COMPARE_FORMAT);
        }
    }

private:
    PinName forward_pin = PC_13;
    PinName reverse_pin = PC_13;
};
