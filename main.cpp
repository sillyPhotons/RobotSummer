#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include <main.h>
#include <vector>
#include <numeric>
#include <Bitmap.h>

Motor left_motor = Motor(MOTOR_LF, MOTOR_LB);
Motor right_motor = Motor(MOTOR_RF, MOTOR_RB);
NewPing sonar(TRIG, ECHO, 200);
NewPing sonar2(TRIG2, ECHO2, 200);

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void dump();
bool align();
bool checkLine();
void PID(float L, float R);
void pick_up_can(bool correction);
float detect_1KHz(int num_samples);
void new_direction(int travel_time);
bool linear_search(short search_radius, int travel_time);
void run1_for_ms(Motor *motor1, short speed, unsigned int ms);
bool search(short max_distance, short spin_time, short increments);
bool run2_for_ms(Motor *motor1, Motor *motor2, short speed1, short speed2, unsigned int ms);

/*
    Only Cans within max_distance will be detected
    - May change with every loop iteration
*/
int max_distance = MAX_DISTANCE;
int l_or_r = 1;
bool line_following = false;
bool found = false;
const int search_time = 7000;
const int align_time = 1000;
unsigned int time_for_tape = 0;

void setup()
{
    Serial1.begin(115200);

    delay(100);
    pinMode(LED_DISPLAY, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.drawBitmap(0, 0, fizzBitMap, 128, 64, WHITE);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    delay(200);
    display.clearDisplay();
    display.setCursor(0, 0);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(R_SENSOR, INPUT);
    pinMode(L_SENSOR, INPUT);
    pinMode(MOTOR_RF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_LB, OUTPUT);

    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}

bool align()
{
    int t = HAL_GetTick();
    int preError = 0;

    // aligning
    while (HAL_GetTick() - t < align_time)
    {
        int cm = sonar.ping_cm(); // take distance measurement
        int error = cm - TARGET_DISTANCE;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("ERROR: ");
        display.println(error);

        // move forward
        if (error > 0)
        {
            left_motor.run_motor(20);
            right_motor.run_motor(20);
        }

        else if (error < 0)
        {
            float P = 1.2 * error;
            float D = 0 * (error - preError);
            float adj = P + D;

            /* 
                The following depends on you. My left motor is always 
                slower than the right when going in reverse. So here I try 
                to apply various corrections 
            */
            Lspeed = constrain(adj, -1 * 50, 40);
            Rspeed = constrain(adj, -1 * 50, 40);

            if (Lspeed > -1 * 40 && Lspeed < 0)
            {
                Lspeed = map(Lspeed, -50, 0, -47, -30);
            }
            left_motor.run_motor(Lspeed);
            right_motor.run_motor(Rspeed);
            display.println(Lspeed);
            display.println(Rspeed);
            display.display();
        }

        prevError = error;
    }

    int cm = sonar.ping_cm(); // take distance measurement
    int error = cm - TARGET_DISTANCE;
    bool complete = false;
    if (abs(error) < 3)
    {
        complete = true;
    }
    return complete;
}

bool linear_search(short search_radius, int travel_time)
{

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("LINE SEARCH");
    display.display();

    int t = HAL_GetTick();

    // searching
    while (HAL_GetTick() - t < travel_time)
    {
        int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
        int cm = sonar.ping_cm();   // take distance measurement

        // if cm2 is 0, then that means object is outside of detection range
        if (cm2 == 0)
        {
            cm2 = 200;
        }
        if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 40)
        {
            Serial1.println("FOUND!");
            found = true;
            break;
        }
        else
        {
            run2_for_ms(&left_motor, &right_motor, 40, 40, 100);
            run2_for_ms(&left_motor, &right_motor, 0, 0, 100);
        }
    }
    return found;
}

/**
  * Searches within the search radius for any objects
  * @param search_radius: centimeters
  * @param l_or_r: 1 is right turn search, -1 is left turn search
  */
bool search(int search_radius, int l_or_r)
{
    int t = HAL_GetTick();

    // searching
    while (HAL_GetTick() - t < search_time)
    {
        int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
        int cm = sonar.ping_cm();   // take distance measurement

        // if cm2 is 0, then that means object is outside of detection range
        if (cm2 == 0)
        {
            cm2 = 200;
        }

        // object is within search radius
        // error will equal -1*TARGET_DISTANCE if the object is out of range
        // the top sensor detects at least twice the distance
        // cm2 has minimum 40 **THIS IS CAUSE MY SENSOR WILL NEVER RETURN VALUE LESS THAN 20**!!
        if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 50)
        {
            found = true;
            break;
        }
        else
        {
            found = false;
            if (l_or_r == 1)
            {
                run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 150);

                Serial1.print("RIGHT with radius: ");
                Serial1.println(search_radius);
            }
            else
            {
                run2_for_ms(&left_motor, &right_motor, 35, -35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 150);

                Serial1.print("LEFT with radius: ");
                Serial1.println(search_radius);
            }
            // display.clearDisplay();
            // display.setCursor(0, 0);
            // display.print("SEARCH with radius: ");
            // display.println(search_radius);
            // display.display();
        }
    }
    return found;
}

/**
 * Run a two motor at the given speed for the given number of milliseconds
 * @param motor1, motor2: motor objects
 * @param speed1, speed2: speed in [-100, 100]
 * @param ms: number of milliseconds the motors will run for
 * @returns: a boolean. True if the action was completed, false if it was
 * interrupted
 * 
 * 
 * TODO: Due to momentum, the robot can't stop right away when it passes tape.
 *       What can we do?
*/
bool run2_for_ms(Motor *motor1, Motor *motor2, short speed1, short speed2, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor1->run_motor(speed1);
    motor2->run_motor(speed2);
    while (HAL_GetTick() - current_tick < ms)
    {
        if ((speed1 > 0 && speed2 > 0) || (speed1 < 0 && speed2 < 0))
        {
            short L = analogRead(L_SENSOR);
            short R = analogRead(R_SENSOR);

            if (L > SETPOINT || R > SETPOINT)
            {
                if (speed1 > 0 && speed2 > 0)
                {
                    motor1->run_motor(-10);
                    motor2->run_motor(-10);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    motor1->run_motor(-50);
                    motor2->run_motor(-50);
                    delay(200);
                    pwm_stop(ARM_SERVO);
                    new_direction(1000);
                    return false;
                }
                else
                {
                    motor1->run_motor(10);
                    motor2->run_motor(10);
                    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
                    delay(500);
                    motor1->run_motor(50);
                    motor2->run_motor(50);
                    delay(200);
                    pwm_stop(ARM_SERVO);
                    new_direction(1000);
                    return false;
                }
            }
        }
    }
    motor2->run_motor(0);
    motor1->run_motor(0);
    return true;
}

/**
 * Run a single motor at the given speed for the given number of 
 * milliseconds
 * @param motor: motor object 
 * @param speed: speed in [-100, 100]
 * @param ms: number of milliseconds the motor will run for
*/
void run1_for_ms(Motor *motor, short speed, unsigned int ms)
{
    unsigned int current_tick = HAL_GetTick();
    motor->run_motor(speed);
    while (HAL_GetTick() - current_tick < ms)
    {
        short L = analogRead(L_SENSOR);
        short R = analogRead(R_SENSOR);

        if (L > SETPOINT || R > SETPOINT)
        {
            return;
        }
    }
    motor->run_motor(0);
    return;
}

void new_direction(int travel_time)
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 1000;
    while (HAL_GetTick() - t < turns)
    {
        left_motor.run_motor(-55);
        right_motor.run_motor(35);
    }
    linear_search(50, travel_time);
}

/**
 * Checks if robot sees the line or not.
 * If yes, PID line following.
 * If no, sharp turn based on previous location to find the line
 * @returns: true if complete
*/
bool checkLine()
{
    short L = analogRead(L_SENSOR);
    short R = analogRead(R_SENSOR);
    // display.clearDisplay();
    // display.setCursor(0, 0);
    // display.println("FOLLOWING ");
    // display.println(L);
    // display.println(R);
    // display.display();

    if (L > SETPOINT)
    {
        L = SETPOINT;
    }
    if (R > SETPOINT)
    {
        R = SETPOINT;
    }
    if (L < SETPOINT && R < SETPOINT)
    {
        if (((L / R > 2.0) || (R / L > 2.0)) && ((L > 250) || R > 250))
        {
            PID(L, R);
        }

        else
        {
            if (prevError > 0)
            {
                right_motor.run_motor(LINE_FOLLOW_SPEED);
                left_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
            }
            else if (prevError < 0)
            {
                right_motor.run_motor(-1.0 * LINE_FOLLOW_SPEED);
                left_motor.run_motor(LINE_FOLLOW_SPEED);
            }
        }
    }
    // ONLY FOR FOR ENDING
    else if (L == SETPOINT && R == SETPOINT)
    {
        if (line_following)
        {
            return true;
        }
    }
    else
    {
        PID(L, R);
        return false;
    }

    return false;
}

/**
 * Computes the adjustment for the motors based on how far off the sensors
 * are from the line and changes motor speed.
 * @param L analog reading from left TCRT5000
 * @param R analog reading from right TCRT5000
*/
void PID(float L, float R)
{

    float R_diff = abs(R - SETPOINT) / ((R + SETPOINT) / 2.0);
    float L_diff = abs(L - SETPOINT) / ((L + SETPOINT) / 2.0);

    if (R_diff + L_diff < 0.10)
    {
        error = 0.0;
    }
    else
    {
        if (R_diff > 0.05 && L_diff < 0.05)
        {
            error = -1;
        }
        else if (R_diff < 0.05 && L_diff > 0.05)
        {
            error = 1;
        }
        else
        {
            error = (prevError / abs(prevError)) * 5;
        }
    }

    prevError = error;

    right_motor.run_motor(Rspeed);
    left_motor.run_motor(Lspeed);
}

void pick_up_can(bool correction)
{
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    if (correction)
    {
        run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
    }
    delay(1000);

    bool mot_1 = run2_for_ms(&right_motor, &left_motor, 100, 100, 500);
    if (!mot_1)
    {
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_H_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_2 = run2_for_ms(&right_motor, &left_motor, 50, 50, 300);
    if (!mot_2)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    bool mot_3 = run2_for_ms(&right_motor, &left_motor, 50, 50, 200);
    if (!mot_3)
    {
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(300);
        return;
    }
    bool mot_4 = run2_for_ms(&left_motor, &right_motor, -80, -80, 200);
    if (!mot_4)
    {
        return;
    }
    run2_for_ms(&left_motor, &right_motor, -100, -100, 300);
}

/**
 * Dumps the cans stored in the robot bin. Won't work if the battery isn't 
 * full
*/
void dump()
{
    dumped = true;
    pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
    delay(500);
    run2_for_ms(&left_motor, &right_motor, 0, 0, 10);
    pwm_start(BIN_SERVO, 50, BIN_UP, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
    delay(3000);
    pwm_stop(BIN_SERVO);
    pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
    delay(500);
    pwm_stop(ARM_SERVO);
}

/**
 * Event loop
*/
bool recycle = false;
int time_elapsed = 0;
void loop()
{
    // run2_for_ms(&left_motor, &right_motor, 20, 20, TAPE_TIME);

    // run2_for_ms(&left_motor, &right_motor, 20, 20, 50);

    // found = search(50, l_or_r);
    // if (found)
    // {
    //     int complete = align();
    //     if (!complete)
    //     {
    //         found = false;
    //     }
    //     if (complete)
    //     {
    //         pick_up_can(true);
    //         delay(1000);
    //     }
    // }
    // else {
    //     new_direction(3000);
    // }

    // checkLine();

    // float intensity = detect_1KHz(100);
    // Serial1.println(intensity);
    // float L = analogRead(L_SENSOR);
    // float R = analogRead(R_SENSOR);
    // display.clearDisplay();
    // display.setCursor(0,0);
    // display.println(L);
    // display.println(R);
    // display.display();
    // Serial1.println(L);
    // Serial1.println(R);

    // Get time since start in ms
    time_elapsed = HAL_GetTick() - start_time;

    // Fixed time to travel on tape
    if (time_elapsed < TAPE_TIME)
    {
        left_motor.run_motor(35);
        right_motor.run_motor(35);
    }
    // After tape start search
    else if (time_elapsed < TOTAL_TIME - HOMING_TIME)
    {
        found = search(50, l_or_r);
        if (!found)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("New Direction");
            display.display();
            new_direction(3000);
        }
        if (found)
        {
            int complete = align();
            if (!complete)
            {
                found = false;
                // l_or_r *= -1;
            }
            if (complete)
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.println("Aligned");
                display.display();
                pick_up_can(true);
                delay(1000);
            }
        }
    }
    else if (!line_following && !dumped)
    {   
        pwm_stop(ARM_SERVO);
        pwm_stop(BIN_SERVO);
        short L = analogRead(L_SENSOR),
              R = analogRead(R_SENSOR);

        if (L < SETPOINT && R < SETPOINT)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("TO TAPE");
            display.println(L);
            display.println(R);
            display.display();
            left_motor.run_motor(30);
            right_motor.run_motor(30);
        }

        else if (L >= SETPOINT || R >= SETPOINT)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("TAPE FOUND");
            display.display();

            while (true)
            {
                L = analogRead(L_SENSOR);
                R = analogRead(R_SENSOR);
                if (L >= SETPOINT)
                {
                    if (R < SETPOINT)
                    {
                        break;
                    }
                }
                else if (R >= SETPOINT)
                {
                    if (L < SETPOINT)
                    {
                        break;
                    }
                }
                run2_for_ms(&left_motor, &right_motor, 0, 0, 75);
                run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
                run2_for_ms(&left_motor, &right_motor, 0, 0, 75);
            }
            left_motor.run_motor(0);
            right_motor.run_motor(0);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("1 SENSOR ON TAPE");
            display.println(L);
            display.println(R);
            display.display();
            line_following = true;
            time_for_tape = HAL_GetTick();
        }
    }
    else if (line_following && !recycle)
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Over Time: ");
        display.println(time_elapsed - TOTAL_TIME);
        display.display();

        bool end = checkLine();
        unsigned int t_now = HAL_GetTick();

        if (end && t_now - time_for_tape > 4000)
        {
            unsigned int t = HAL_GetTick();
            while ((HAL_GetTick() - t) < BACK_UP_TIME)
            {   
                left_motor.run_motor(-50);
                right_motor.run_motor(-40);
            }
            left_motor.run_motor(0);
            right_motor.run_motor(0);
            recycle = true;
        }
    }
    else if (recycle)
    {
        dump();
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
};