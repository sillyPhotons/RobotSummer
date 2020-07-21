#include <main.h>
#include <Search.h>
#include <Hardware.h>
#include <Home.h>

void setup()
{
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

short state = entering;
Home Home_Manager = Home(4000);
Search Search_Manager = Search(10, 5, 6);
int time_elapsed = 0;

/**
 * Event loop
*/
void loop()
{
    time_elapsed = HAL_GetTick() - start_time;

    if (time_elapsed < TAPE_TIME)
    {
        state = entering;
    }
    else if (time_elapsed < TOTAL_TIME - HOMING_TIME)
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
        left_motor.run_motor(35);
        right_motor.run_motor(35);
        break;

    case searching:
        Search_Manager.loop();
        break;

    case homing:
        Home_Manager.loop();
        break;
    }
};