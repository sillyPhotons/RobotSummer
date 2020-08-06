#include <main.h>
#include <Search.h>
#include <Hardware.h>
#include <Home.h>

// Timing variables
short competition_mode = 1;

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
    display.setTextSize(3);
    delay(200);
    display.clearDisplay();
    display.setCursor(0, 0);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(R_SENSOR, INPUT);
    pinMode(L_SENSOR, INPUT);
    pinMode(R_SENSOR2, INPUT);
    pinMode(L_SENSOR2, INPUT);
    pinMode(MOTOR_RF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_LB, OUTPUT);

    int mode = analogRead(PGAIN);
    if (mode > 500)
    {
        competition_mode = 0;
        pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
        delay(500);
    }
    else
    {
        competition_mode = 1;
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(500);
        pwm_stop(ARM_SERVO);
    }
    // check_crossed_tape will be called 100 times every second
    MyTim.setOverflow(100, HERTZ_FORMAT);
    MyTim.attachInterrupt(check_sensors);
    MyTim.resume();
}

short state = entering;

// 4 second delay before recognizing both sensor high as end point
Home Home_Manager = Home(1000);

Search Search_Manager = Search(12, 5, 7);
int time_elapsed = 0;
unsigned short counter = 1;

bool found = false;
void loop()
{
    time_elapsed = HAL_GetTick() - start_time;
    switch (competition_mode)
    {
    case 1:
    {
        if (counter % 10 == 0)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(time_elapsed / 1000.0);
            display.display();
            counter = 1;
        }

        if (time_elapsed < TAPE_TIME && !found)
        {
            state = entering;
        }
        else if (time_elapsed < TOTAL_TIME - TAPE_TIME - HOMING_TIME)
        {
            state = searching;
        }
        else {
            MyTim.detachInterrupt();
            state = homing;
        }

        switch (state)
        {
        case entering:
            found = Search_Manager.enter_arena();
            crossed = false;
            break;

        case searching:
            if (crossed)
            {
                crossed_tape();
            }
            Search_Manager.loop();
            if (crossed)
            {
                crossed_tape();
            }
            break;

        case homing:
            Home_Manager.loop();
            break;
        }
        counter += 1;
        break;
    }
    case 0:
    {
        if (time_elapsed < TAPE_TIME / 3)
        {
            state = spin;
        }
        else if (time_elapsed > TAPE_TIME / 3 && state != complete)
        {
            state = retrieve;
        }

        switch (state)
        {
        case spin:
            run_both(-55, 35, 55, false);
            run_both(0, 0, 95, false);
            break;

        case retrieve:
            run_both(100, 100, 550, false);
            pwm_start(ARM_SERVO, 50, ARM_H_UP1, MICROSEC_COMPARE_FORMAT);
            run_both(100, 100, 200, false);
            pwm_start(ARM_SERVO, 50, ARM_H_UP1 - 500, MICROSEC_COMPARE_FORMAT);
            run_both(100, 100, 200, false);
            run_both(30, 30, 200, false);

            state = complete;
            break;

        case complete:
            run_both(0, 0, 50, false);
            break;
        }
        break;
    }
    }
};