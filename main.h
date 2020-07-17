// Timing variables
const int start_time = HAL_GetTick();
const int TAPE_TIME = 3000;
const int TOTAL_TIME = 60000;
const int HOMING_TIME = 10000;
const int BACK_UP_TIME = 1000;
// PID Control variables
u_int32_t SETPOINT = 500;
const int GAIN = 5;
int prevError = 0;
const int LINE_FOLLOW_SPEED = 30;
int max_adjustment_speed = LINE_FOLLOW_SPEED + 10;
float P, D, error = 0;
int Lspeed = LINE_FOLLOW_SPEED;
int Rspeed = LINE_FOLLOW_SPEED;

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
