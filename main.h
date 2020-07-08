// Timing variables
const int TOTAL_TIME = 60000;
int HOMING_TIME = 10000;

// PID Control variables
const int SETPOINT = 250;
const int GAIN = 5;
int last_error = 0;

class Arm
{
public:
    Arm(PinName arm_servo): pin(arm_servo){};
    void up_to_down(){
        pwm_start(PA_6, 50, 1000, MICROSEC_COMPARE_FORMAT);
    }
private:
    PinName pin;
};
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

class Phototransistor
{
public:

    Phototransistor(int photo_pin, double target_frequency) : photo_pin(photo_pin), target_frequency(target_frequency)
    {
        
        ;
    };

    int get_pin()
    {
        return photo_pin;
    }

private:
    int photo_pin = 0;
    double target_frequency;
};