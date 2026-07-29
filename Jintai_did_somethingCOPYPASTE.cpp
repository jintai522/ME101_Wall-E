#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorDoor3 = motor(PORT3, false);
motor MotorCompress9 = motor(PORT9, false);
motor LeftMotor = motor(PORT7, false);
motor RightMotor = motor(PORT12, true);
distance Distance2 = distance(PORT2);
optical Optical8 = optical(PORT1);
motor MotorScoop8 = motor(PORT8, false);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

// Converts a color to a string
const char* convertColorToString(color col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


// Convert colorType to string
const char* convertColorToString(colorType col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration
#include "iq_cpp.h"

/*
GROUP 20:
  Daniel Chen, Jintai Li
  Luke Liu, Sam Zhang
PROJECT:
  VEX-E
*/

using namespace vex;

const double WHEEL_C = 200;//mm

const double ROTATE_KP = 1.8;
const double STRAIGHT_KP = 2;

const double SCOOPSPEED = 10;

const double COMPRESS_DEGREE = -4000;
const int COMPRESS_SPEED = 75;
const double compressCurrentMax = 0.4;


motor_group driveMotor(LeftMotor,RightMotor);

void configureAllSensors()
{
  Brain.Screen.print("Calibrating...");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating())
  {}
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  BrainInertial.resetRotation();
  Optical8.setLight(ledState::on);
}
/*
PID Sources
https://www.youtube.com/watch?v=KMSJviT2jI8
https://lucykim0907-byte.github.io/Pid-build-log-by-Jaehyeon-Shin/pid_portfolio.html
P is the proportional, which is error
  (how far off)
*/


void rotateRobot(double targetAngle, int speed)
{
  double error = 676; // change if needed
  double startAngle = BrainInertial.rotation(degrees);
  double targetAbsolute = startAngle + targetAngle;
  while (fabs(error) > 0.3) // get at 0.3 degree of target
  {
    double currentAngle = BrainInertial.rotation(degrees);
    error = targetAbsolute - currentAngle;

    double power = ROTATE_KP * error;
    if (power > speed)
    {
      power = speed;
    }
    else if (power < -speed)
    {
      power = -speed;
    }

    if (power > 0 and power < 5) 
    {
      power = 5;
    }
    else if (power < 0 and power > -5)
    {
      power = -5;
    }

    if (power > 0)
    {
      LeftMotor.spin(forward, power, percent);
      RightMotor.spin(reverse, power, percent);
    }
    else if (power < 0)
    {
      double absPower = -power;
      LeftMotor.spin(reverse, absPower, percent);
      RightMotor.spin(forward, absPower, percent);
    }
    else
    {
      driveMotor.stop(brake);
    }
  }
  driveMotor.stop(brake);
}

bool scoopDetect(double min, double max, int & run_state)
{
  if(Distance2.objectDistance(mm) < max and Distance2.objectDistance(mm) > min)
  {
    Brain.Screen.print("Object Detected");
    run_state = 2;
    return true;
  }
  else 
  {
    return false;
  }
}

void straightDrive(double distance, int speed,int & run_state)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while 
  (
  ((fabs(LeftMotor.position(turns)*WHEEL_C)+fabs(RightMotor.position(turns)*WHEEL_C))/ 2.0 < distance)
  and !(scoopDetect(10,30,run_state))
  )
  {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%6.2f  L:%.2f  R:%.2f  Piston:%.2f\n",
    BrainInertial.heading(degrees),
    BrainInertial.rotation(degrees));

  
    double heading = BrainInertial.rotation(degrees);
    double error = 0 - heading; // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;

    LeftMotor.spin(forward, speed + correction, percent);
    RightMotor.spin(forward, speed - correction, percent);
  }
  driveMotor.stop(brake);
}

/*void reverseStraightDrive(double distance, int speed,int & run_state)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while 
  (
  ((fabs(LeftMotor.position(turns)*WHEEL_C)+fabs(RightMotor.position(turns)*WHEEL_C))/ 2.0 < distance)
  and !(scoopDetect(10,30,run_state))
  )
  {
    double heading = BrainInertial.rotation(degrees);
    double error = fabs(0 - heading); // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;

    LeftMotor.spin(forward, speed + correction, percent);
    RightMotor.spin(forward, speed - correction, percent);
  }
  driveMotor.stop(brake);
}
*/

void scoopRobot ( double scooperSpeed, int & run_state, int & scoop_count)
{
  straightDrive(500,30,run_state);

  MotorScoop8.spin(forward,scooperSpeed,percent);
  wait(4, seconds);
  MotorScoop8.stop();

  wait(0.5,seconds);

  straightDrive(500, 100,run_state);

  MotorScoop8.spin(reverse,47,percent);
  wait(1, seconds);
  MotorScoop8.stop();

  run_state = 3; 
}

void movedoor (int speed, directionType motorDir)
{
  double degree = 0; 
  MotorDoor3.spin(motorDir, speed, percent);
  while (MotorDoor3.current(amp) < 0.2)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%6.2f  L:%.2f  R:%.2f  Piston:%.2f\n",
    Brain.timer(seconds),
    MotorDoor3.current(amp));
    wait(100, msec);
  }
  degree = fabs(MotorDoor3.position(degrees));
  MotorDoor3.stop(brake);
}


void compress(double degree, int speed, int & run_state)
{
  MotorCompress9.resetPosition();
  MotorCompress9.spin(reverse, speed, percent);
  while (MotorCompress9.current(amp) < compressCurrentMax)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%6.2f  L:%.2f  R:%.2f  Piston:%.2f\n",
    Brain.timer(seconds),
    MotorCompress9.current(amp));
    wait(100, msec);
  }
  degree = fabs(MotorCompress9.position(degrees));
  MotorCompress9.stop(brake);

  wait(1, seconds);

  MotorCompress9.resetPosition();
  MotorCompress9.spin(forward, speed, percent);
  while (fabs(MotorCompress9.position(degrees)) <= fabs(degree))
  //while (MotorCompress9.current(amp) < 0.2)
  {}
  MotorCompress9.stop(brake);

  
  run_state = 0;
}



void userInter(int & run_state)
{
  Brain.Screen.clearScreen();
  //out put whats its doing during the state
}


int main() 
 /*
    0 close program
    1 run pattern
    2 run scooping
    3 run compress
    4 run dumping
  */

{
  configureAllSensors();
  int run_state = 1;
  int scoop_count = 0; 
 
  while (!Brain.buttonCheck.pressing()) 
  {}
  while (Brain.buttonCheck.pressing())
  {}

  while(run_state != 0)
  {
    if (run_state == 1) 
    {
      straightDrive(1000, 30,run_state);
    }
    else if (run_state == 2)
    {
     scoopRobot(SCOOPSPEED,run_state, scoop_count);
    }
    else if (run_state == 3)
    {
       compress(COMPRESS_DEGREE, COMPRESS_SPEED, run_state);
     
    }
  }

  Optical8.setLight(ledState::off);
  Brain.programStop();
  return EXIT_SUCCESS;
}
