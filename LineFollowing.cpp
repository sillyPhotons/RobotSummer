#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "LFheader.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void PID(double L, double R);
void runMotors(double LF, double LB, double RF, double RB);
void checkLine();

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
  
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(KP_POT, INPUT);

  pinMode(R_SENSOR, INPUT);
  pinMode(L_SENSOR, INPUT);

  pinMode(MOTOR_RF, OUTPUT);
  pinMode(MOTOR_RB, OUTPUT);
  pinMode(MOTOR_LF, OUTPUT);
  pinMode(MOTOR_LB, OUTPUT);  
}

double P, I, D, error;
double prevError = 0.0;
double Lspeed = INITIAL_SPEED;
double Rspeed = INITIAL_SPEED;
//double Kp = 0;

void loop() {
  if (!digitalRead(SWITCH)) {
    runMotors(0,0,0,0);
    //sample code for tuning PID constants
    /*
    Kp = analogRead(KP_POT);
    display.clearDisplay();
    display.print("Kp: ");
    display.println(Kp);
    display.display();
    */
  } else {
    checkLine();
  }
}

/*
Checks if robot sees the line or not.
If yes, PID line following.
If no, sharp turn based on previous location to find the line
*/
void checkLine() {
  double L = analogRead(L_SENSOR);
  double R = analogRead(R_SENSOR);
  if (L < THRESHOLD & R < THRESHOLD) {
    if (prevError > 0) {
      runMotors(0, 1000, 1000, 0);
    } else {
       runMotors(1000, 0, 0, 1000);
    }
  } else {   
      PID(L,R);
  }  
}

/*
Computes the adjustment for the motors based on how far off the sensors
 are from the line and changes motor speed.

 @param L analog reading from left TCRT5000
 @param R analog reading from right TCRT5000
*/
void PID(double L, double R) {
  error = THRESHOLD - R - (THRESHOLD - L);
  P = error;
  I += error;
  D = error - prevError;
  prevError = error;
  double adj = Kp*P + Ki*I + Kd*D;
  Lspeed = constrain(Lspeed - adj , 200, 1000);
  Rspeed = constrain(Rspeed + adj, 200, 900);
  runMotors(Lspeed, 0, Rspeed, 0);
}

/*
Sets motor speed
@param LF forwards speed of left motor
@param LB backwards speed of left motor
@param RF forwards speed of right motor
@param RB backwards speed of right motor
*/
void runMotors(double LF, double LB, double RF, double RB) {
  pwm_start(MOTOR_RF, PWMFREQ, RF,RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_LF, PWMFREQ, LF,RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_RB, PWMFREQ, RB,RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_LB, PWMFREQ, LB,RESOLUTION_10B_COMPARE_FORMAT);
}
