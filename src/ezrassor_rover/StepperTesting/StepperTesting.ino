
/*

  Stepper motor Testing Made for FSI 

 */

#include "DRV8825.h"
// For RAMPS 1.4 pins defined below


// Pins for X
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

// Pins for Y
#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

// Pins for Z
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



// initialize the stepper library on the pins you have the stepper motor on, on the ramps board these have been defined above for you
// this will test one motor 
DRV8825 frontLeft(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, 200 * 64);
DRV8825 frontRight(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, 200 * 64);
// Back  of Rover --- E0/Z
DRV8825 backLeft(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, 200 * 64);
DRV8825 backRight(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, 200 * 64);
// global variables -- can be changed in a later version 
long idle_speed = 1;
long time = millis();
long time1 = 0;

// global checkers for starting or stopping 
int on = 0;
int stop = 0;



void setup() {
  // initialize the serial port to 115200:
  Serial.begin(115200);
  frontLeft.set_enabled(true);
  frontLeft.set_direction(false);
  frontLeft.set_speed(0);

  frontRight.set_enabled(true);
  frontRight.set_direction(false);
  frontRight.set_speed(0);
  // setup the LED to give feed back aswell (not needed but is nice to have)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

}

// will run forever Turning the motors
void loop() {

      time1 = millis();
      update_motors();
      Control();
      // Controlls the slow start for the stepper motor checks if 2500 miliseconds have passed and if we want to speed up
      if(time1 - time >= 2500 && on == 1)
      {
        Speedup(idle_speed);
      }
      // controlls the soft stop for the stepper motor checks if 2500 milliseconds have passed and if we want to stop
      if(time1 - time >= 2500 && stop == 1)
      {
        Stop();
      }
}
// Main control switch for checking what command has been passed into the serial port 
void Control()
{
  // checks if the serial port is avaliable if it is allow the user to specify what command they want to run 
 if (Serial.available())
      {

        int command = Serial.read();
        
        switch(command)
        {
          case 's':
            Stop();
            stop = 1;
            break;
          case 'f': 

            on = 1;
            long tmp1 = 1;

            Speedup(tmp1);

            
            
            Serial.println("Exited speedup");

            digitalWrite(LED_BUILTIN, HIGH);
            break;

        }
      }

}
// Speed up function used to speed up the stepper motors from 0 to a set variable, takes in tmp as the starting variable for the speed up
void Speedup(long tmp)
{    
    Serial.print(tmp);
    
    if(tmp >= 60)
      {
          on = 0;
          return;
      }
      time = millis();  
      Serial.println("Running Speedup");
      frontLeft.set_direction(false);
      frontRight.set_direction(false);
      frontLeft.set_speed(tmp);
      tmp = tmp*2;
      idle_speed = tmp;
      frontRight.set_speed(tmp);
      Serial.println(tmp);

}

// The stop function to be called to slowly stop the motors 
void Stop()
{    

    
    if(idle_speed <= 0)
    {
      idle_speed = 1;
      stop = 0;
      frontLeft.set_speed(0);
      frontRight.set_speed(0);
      return;
    }
      time = millis();  
      Serial.println("Running Stop");
      idle_speed = idle_speed/2;
      frontLeft.set_speed(idle_speed);
      frontRight.set_speed(idle_speed);
      Serial.println(idle_speed);

}
// Updates the motors by calling the function update for each motor
void update_motors()
{

  frontLeft.update();
  frontRight.update();
  backLeft.update();
  backRight.update();
}


