//OLED display setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

//Tape sensors
#define R_SENSOR PB1
#define L_SENSOR PB0
#define THRESHOLD 350.0

//Motors
#define MOTOR_LF PB_8
#define MOTOR_LB PB_9
#define MOTOR_RF PA_0
#define MOTOR_RB PA_1
#define PWMFREQ 700
#define INITIAL_SPEED 600.0

#define KP_POT PA_5
#define SWITCH PB5

//PID constants
#define Ki 0.0
#define Kd 30
#define Kp 0.2
