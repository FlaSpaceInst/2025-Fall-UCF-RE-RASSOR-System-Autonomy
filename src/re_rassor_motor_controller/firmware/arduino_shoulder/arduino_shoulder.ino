// Copyright 2025 UCF RE-RASSOR
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include "DRV8825.h"
// For RAMPS 1.4
/*

43-1 reduction 
big wheel = 
little wheel = 15 1/2
*/

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

// extruder 1
#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24

// extruder 2
#define E1_STEP_PIN 36
#define E1_DIR_PIN 34
#define E1_ENABLE_PIN 30



/* wheel based on arduino pin position
    E1 = Front Left
    E0 = Back Left
    Y = Front Right
    X = Back Right
*/

DRV8825 backRight(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, 100 * 64);
DRV8825 frontRight(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, 100 * 64);
DRV8825 frontLeft(E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, 100 * 64);
DRV8825 backLeft(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, 100 * 64);

// Free driver: Z
DRV8825 FreeDriver(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, 100 * 64);


enum COMMANDS {
  STOP = 0x00,
  HALT = 0xff,
  DUMP = 0x01, // may break code if this overlaps with wheel
  DIG = 0x02,
  FRONT = 0x05,
  BACK = 0x06,
  RAISE = 0X07,
  LOWER = 0X08
};

byte last_command = STOP;

long last_command_time = 0;   // ms since last command
long command_timeout = 1000;  // ms to wait for next command before stopping

// change this number to adjust the max rpms of the stepper motors
long maxSpeed = 16;

// The speed of the motors per side
long idle_left_speed = 1;
long idle_right_speed = 1;

long time = millis();
long timeout = 0;
long time1 = 0;

// direction that indicates what side is going where
bool rs = false;  // right side
bool ls = false;  // left side

// global checkers for starting or stopping
int on = 0;
int stop = 0;

// variable to control slow stop/accelerate
int slowSpeed = 1;

// indicates whether we go slow, medium, fast, or not moving with +/- for direction
int leftSpeed = 0;
int rightSpeed = 0;



void setup() {

  // use USB on serial 115200
  Serial.begin(115200);


  // set up the LED for ability to see if recieving commands
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // wheels right/left initial
  frontRight.set_enabled(true);
  frontRight.set_direction(false);
  frontRight.set_speed(0);

  backRight.set_enabled(true);
  backRight.set_direction(false);
  backRight.set_speed(0);

  // shoulders front/back initial
  frontLeft.set_enabled(true);
  frontLeft.set_direction(false);
  frontLeft.set_speed(0);

  backLeft.set_enabled(true);
  backLeft.set_direction(false);
  backLeft.set_speed(0);
}

long last_frequency_check_time = 0;
long counter = 0;


void loop() {

  time1 = millis();
  update_motors();


  read_serial();


  // Controlls the slow start for the stepper motor checks if a certain amount of miliseconds have passed and if we want to speed up
  if (time1 - time >= 750 && on == 1) {
    Speedup(ls, rs);
  }
  // controlls the soft stop for the stepper motor checks if a certain amount of have passed and if we want to stop
  if (time1 - time >= 750 && stop == 1) {
    Stop();
  }
  // call stop function if we get stuck in a loop and it wont slow down after 2500 miliseconds
  if (timeout == 750) {
    stop = 1;
    on = 0;
    Stop();
  }
}
// updates the motors
void update_motors() {

  backRight.update();
  frontRight.update();
  backLeft.update();
  frontLeft.update();
}
// getting hung up and sending stop when we dont want to stop
// checks for commands being sent over the Serial port to the arduino/Ramps board
void read_serial() {
  if (Serial.available()) {
    
    last_command = Serial.read();
    
    // The below comments are for debugging, comment the line above and decomment the below to debug [ctrl + /]

    // String command = Serial.readStringUntil('\n');
    // if (command == "FRONT")
    //   last_command = FRONT;
    // if (command == "BACK")
    //   last_command = BACK;
    // if (command == "RAISE")
    //   last_command = RAISE;
    // if (command == "LOWER")
    //   last_command = LOWER;
    // if (command == "DIG")
    //   last_command = DIG;
    // if (command == "DUMP")
    //   last_command = DUMP;
    // if (command == "STOP")
    //   last_command = STOP;

    switch (last_command) {
      case STOP:
        stop = 1;
        on = 0;
        Stop();
        break;

      case DUMP:
        frontLeft.set_direction(true);
        frontLeft.set_speed(10);
        backLeft.set_direction(false);
        backLeft.set_speed(10);
        break;
      case DIG:
        frontLeft.set_direction(false);
        frontLeft.set_speed(10);
        backLeft.set_direction(true);
        backLeft.set_speed(10);
        break;
      case FRONT:
        frontRight.set_direction(false);
        frontRight.set_speed(10);
        break;

      case BACK:
        frontRight.set_direction(true);
        frontRight.set_speed(10);
        break;

      case RAISE:
        backRight.set_direction(false);
        backRight.set_speed(10);
        break;

      case LOWER:
        backRight.set_direction(true);
        backRight.set_speed(10);
        break;
        
      default:
        digitalWrite(LED_BUILTIN, LOW);
        stop = 1;
        on = 0;
        Stop();
        break;
    }
  }
}


// Speed up function used to speed up the stepper motors from 0 to a set variable, takes in tmp as the starting variable for the speed up
void Speedup(bool left, bool right) {

}


// The stop function to be called to slowly stop the motors
void Stop() {
  if (abs(idle_left_speed) <= 1 && abs(idle_right_speed) <= 1) {
    idle_left_speed = 1;
    idle_right_speed = 1;
    slowSpeed = 1;
    stop = 0;
    leftSpeed = 0;
    rightSpeed = 0;
    backRight.set_speed(0);
    frontRight.set_speed(0);
    backLeft.set_speed(0);
    frontLeft.set_speed(0);

    Serial.println("--STOPPED--");
    return;
  }
  time = millis();
  Serial.println("Cutting speed in half...");

  idle_left_speed = idle_left_speed / 2;
  idle_right_speed = idle_right_speed / 2;

  if (idle_left_speed < 0) {
    frontLeft.set_speed(-1 * idle_left_speed);
    backLeft.set_speed(-1 * idle_left_speed);
  } else {
    frontLeft.set_speed(idle_left_speed);
    backLeft.set_speed(idle_left_speed);
  }

  if (idle_right_speed < 0) {
    backRight.set_speed(-1 * idle_right_speed);
    frontRight.set_speed(-1 * idle_right_speed);
  } else {
    backRight.set_speed(idle_right_speed);
    frontRight.set_speed(idle_right_speed);
  }
}