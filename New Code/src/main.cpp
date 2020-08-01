#include <main.h>
#include <Search.h>
#include <Hardware.h>
#include <Home.h>

// Timing variables
const int start_time = HAL_GetTick();
bool competition_mode = true;

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
    pinMode(MOTOR_RF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_LB, OUTPUT);

    int mode = analogRead(PGAIN);
    if (mode > 500)
    {
        competition_mode = false;
        pwm_start(ARM_SERVO, 50, ARM_REST, MICROSEC_COMPARE_FORMAT);
        delay(500);
    }
    else
    {
        pwm_start(BIN_SERVO, 50, BIN_REST, MICROSEC_COMPARE_FORMAT);
        delay(500);
        pwm_stop(BIN_SERVO);
        pwm_start(ARM_SERVO, 50, ARM_UP, MICROSEC_COMPARE_FORMAT);
        delay(500);
        pwm_stop(ARM_SERVO);
    }

#if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
#else
    TIM_TypeDef *Instance = TIM2;
#endif
    HardwareTimer *MyTim = new HardwareTimer(Instance);

    MyTim->setOverflow(120, HERTZ_FORMAT); // 10 Hz
    MyTim->attachInterrupt(check_crossed_tape);
    MyTim->resume();
}

short state = entering;
Home Home_Manager = Home(4000); // 4 second delay before recognizing both sensor high as end point
Search Search_Manager = Search(15, 5, 8);
int time_elapsed = 0;
unsigned short counter = 1;

void loop()
{
    time_elapsed = HAL_GetTick() - start_time;

    switch (competition_mode)
    {
    case true:
    {
        if (counter % 10 == 0)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(time_elapsed / 1000.0);
            display.display();
            counter = 1;
        }

        if (time_elapsed < TAPE_TIME)
        {
            state = entering;
        }
        else if (time_elapsed < TOTAL_TIME - TAPE_TIME - HOMING_TIME)
        {
            state = searching;
        }
        else
        {
            state = homing;
        }

        switch (state)
        {
        case entering:
            Search_Manager.enter_arena();
            crossed = false;
            L_crossed = false;
            R_crossed = false;
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
    case false:
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
            run_both(50, 50, 300, false);
            run_both(30, 30, 200, false);
            run_both(20, 20, 100, false);
            pwm_start(ARM_SERVO, 50, ARM_H_UP1 - 500, MICROSEC_COMPARE_FORMAT);
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