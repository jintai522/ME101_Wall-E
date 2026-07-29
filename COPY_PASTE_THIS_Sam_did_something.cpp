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
motor MotorCompress9 = motor(PORT9, true);
motor LeftMotor = motor(PORT7, false);
motor RightMotor = motor(PORT12, true);
motor Scooper8 = motor(PORT8, false);
optical Optical11 = optical(PORT11);
distance Distance2 = distance(PORT2);


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

const double WHEEL_C = 200; //mm

const double ROTATE_KP = 1.8;
const double STRAIGHT_KP = 2;

const double GREEN_HUE_MIN = 75;
const double GREEN_HUE_MAX = 170;
const double BLUE_HUE_MIN = 190;
const double BLUE_HUE_MAX = 359;

const double ZIGZAG_STEP = 180; //mm

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
  Optical11.setLight(ledState::on);
}

void movedoor (int speed, directionType motorDir)
{
  double degree = 0; 
  MotorDoor3.spin(motorDir, speed, percent);
  while (MotorDoor3.current(amp) < 0.2)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("%6.2f  L:%.2f  R:%.2f  Piston:%.2f\n",
    Brain.timer(seconds),MotorDoor3.current(amp));
    wait(100, msec);
  }
  degree = fabs(MotorDoor3.position(degrees));
  MotorDoor3.stop(brake);
}


void userInter(int & run_state)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  if (run_state == 5)
  {
    Brain.Screen.print("Hi, I'm VEX-E");
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Press The Check");
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Mark to Start!");
  }
  else if (run_state == 1) Brain.Screen.print("Searching");
  else if (run_state == 2) Brain.Screen.print("Scooping");
  else if (run_state == 3) Brain.Screen.print("Compressing");
  else if (run_state == 4) Brain.Screen.print("Returning");
  else if (run_state == 0) Brain.Screen.print("Ending Program");
}

/*PID Sources
https://www.youtube.com/watch?v=KMSJviT2jI8
https://lucykim0907-byte.github.io/Pid-build-log-by-Jaehyeon-Shin/pid_portfolio.html
P is the proportional, which is error how far off)*/
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
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine(1);
    Brain.Screen.print("Object Detected");
    run_state = 2;
    return true;
  }
  else
  {
    return false;
  }
}

void straightDrive(double distance, int speed)
{
  directionType dir;//the direction for rotation
  if (distance >= 0) 
  {
    dir = forward;
  }
  else
  {
    dir = reverse;
  }
  double targetDist = fabs(distance);

  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while 
  (((fabs(LeftMotor.position(turns) * WHEEL_C) + fabs(RightMotor.position(turns) * WHEEL_C)) / 2.0 < targetDist))
  {
    double heading = BrainInertial.rotation(degrees);
    double error = 0 - heading; // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;//how much to sping other way

    if (dir == reverse) 
    {
      correction = -correction;
    }

    int leftPower  = speed + correction;
    int rightPower = speed - correction;
    //clamp max and min
    if (leftPower > 67) leftPower = 67;
    if (leftPower < 6) leftPower = 7;
    if (rightPower > 67) rightPower = 67;
    if (rightPower < 6) rightPower = 7;

    LeftMotor.spin(dir, leftPower, percent);
    RightMotor.spin(dir, rightPower, percent);
  }
  driveMotor.stop(brake);
}

void scoopRobot ( int & run_state, int & scoop_count)
{
  scoop_count++; 
  straightDrive(-350,30);

  Scooper8.spin(forward,30,percent);
  wait(1.9, seconds);
  Scooper8.stop();

  wait(0.5,seconds);

  straightDrive(400, 100);
  straightDrive(100, 67);
  straightDrive(100, 35);

  Scooper8.spin(reverse,47,percent);
  wait(1, seconds);
  Scooper8.stop();

  if (scoop_count > 6)
  {
    run_state = 4; 
  }

  else 
  {
    run_state = 1;
  }
}

bool seesColor(double hueMin, double hueMax, int colour)
{
  double hue = Optical11.hue();
  color col = Optical11.color();
  bool detected = (hue >= hueMin and hue <= hueMax);

  if (detected)
  {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine(1);
    if (colour == 6)
    {
      Brain.Screen.print("Green Detected");
    }
    else if (colour == 7)
    {
      Brain.Screen.print("Blue Detected");
    }
  }
  return detected;
}

void driveUntilGreen(int speed, int & run_state)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();
  while ((!seesColor(GREEN_HUE_MIN, GREEN_HUE_MAX, 6)) and !(scoopDetect(10,36,run_state)))
  {
    double heading = BrainInertial.rotation(degrees);
    double error = 0 - heading; // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;

    LeftMotor.spin(forward, speed + correction, percent);
    RightMotor.spin(forward, speed - correction, percent);

    if (seesColor(BLUE_HUE_MIN, BLUE_HUE_MAX, 7))
    {
      run_state = 4;
    }
  }
  driveMotor.stop(brake);
}

void pathFind(int speed, int & run_state, bool & facingRight)
{
  while (run_state == 1)
  {
    driveUntilGreen(speed, run_state);
    if (run_state == 1)
    { 
      userInter(run_state);

      int turnSign = 0;
      if (facingRight)
      {
        turnSign = -1;
      }
      else
      {
        turnSign = 1;
      }
      rotateRobot(90 * turnSign, 18);
      straightDrive(ZIGZAG_STEP, speed);
      rotateRobot(90 * turnSign, 18);

      facingRight = !facingRight;
    }
  }
}

void returning(int speed, int & run_state, bool & facingRight, double & search_width, double & search_length)
{
  double startwidth = 0;
  double endwidth = 0; 
  double startlength = 0; 
  double endlength = 0; 

  int turnSign = 0;
  if (facingRight)
  {
    turnSign = 1;
  }
  else
  {
    turnSign = -1;
  }
  straightDrive(-140, 30); // first backs up a bit 

  rotateRobot(90 * turnSign, 18);

  startwidth = (fabs(LeftMotor.position(turns) * WHEEL_C) + fabs(RightMotor.position(turns) * WHEEL_C)) / 2.0; 

  driveUntilGreen(speed, run_state); // touches the first green line 

  endwidth = (fabs(LeftMotor.position(turns) * WHEEL_C) + fabs(RightMotor.position(turns) * WHEEL_C)) / 2.0; 

  straightDrive(200, speed); // get pass the green line 

  rotateRobot(90, 18); 

  startlength = (fabs(LeftMotor.position(turns) * WHEEL_C) + fabs(RightMotor.position(turns) * WHEEL_C)) / 2.0;

  driveUntilGreen(speed, run_state); // drive until touches dumping area 
  
  endlength = (fabs(LeftMotor.position(turns) * WHEEL_C) + fabs(RightMotor.position(turns) * WHEEL_C)) / 2.0;

  search_width = endwidth - startwidth; // measures "y" pos/ length between the car to the bottom green tape when it got full midtrack 
  search_length = endlength - startlength; //measures "x" pos / length between car and the dumping place 

  run_state = 3;   // call the compression now 
}

void compressRobot(int speed, double maxAmp, double & wall_dist_temp)
{
  MotorCompress9.resetPosition();
  MotorCompress9.spin(forward, speed, percent);
  while (MotorCompress9.current(amp) < maxAmp) // determines how much the motor can push 
  //this ampere was determined based on trial test and also helps verify design specific..
  {
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("%6.2f  A:%.2f  R:%.2f  Piston:%.2f\n",
    Brain.timer(seconds), MotorCompress9.current(amp));
    wait(100, msec);
  }
  wall_dist_temp = fabs(MotorCompress9.position(degrees));
  MotorCompress9.stop(brake);

  wait(0.5, seconds);

  MotorCompress9.spin(reverse, speed, percent);
  while (fabs(MotorCompress9.position(degrees)- wall_dist_temp) <= 360) // backs up a little bit 
  {}
  MotorCompress9.stop(brake);
}

void dump (int wall_dist_tot, int speed, double max_amp, double & wall_dist_temp, int & scoop_count)
{
  MotorCompress9.spin(forward, speed, percent); 
  while (fabs(MotorCompress9.position(degrees)) < wall_dist_tot - 20 - wall_dist_temp)
  {}
  MotorCompress9.stop(brake); 

  MotorCompress9.spin(reverse, speed, percent); 
  while (MotorCompress9.position(degrees) > 0 && MotorCompress9.current(amp) < max_amp)
  {}
  MotorCompress9.stop(); 
}


void returnStart(int & run_state)
{
  run_state = 0; 
}

void returnsearch(double & search_width, double & search_length, bool & facingRight, int & run_state)
{
  int turnSign = 0;
  if (facingRight)
  {
    turnSign = 1;
  }
  else
  {
    turnSign = -1;
  }

  straightDrive(-search_length, 25);
  rotateRobot(90, 18); 
  straightDrive(200, 25); 
  straightDrive(search_width, 25);
  rotateRobot(90*turnSign, 18); 
  
  run_state = 1;
}


void end_or_search (double & search_width, double & search_length, bool & facingRight,int & scoop_count, int & run_state)
{
 if (scoop_count > 6)
  {
    returnsearch(search_width,search_length,facingRight,run_state);
  } 
  else 
  {
    returnStart(run_state);
  }
  scoop_count = 0; 
}

//VEX-E MAIN 

int main()
{
  configureAllSensors();
  int run_state = 5;
  bool facingRight = true; //assume
  int scoop_count = 5;
  double wall_dist_temp = 0; 
  double search_width = 0; 
  double search_length = 0; 
 
  /*
    0 close program
    1 run pattern
    2 run scooping
    3 run compress
    4 return to start
    5 welcome
  */
  userInter(run_state);
  while (!Brain.buttonCheck.pressing())
  {}
  while (Brain.buttonCheck.pressing())
  {}
  run_state = 1;

  while (run_state != 0)
  {
    userInter(run_state);//Will say what it does
    if (run_state == 1)
    {
      pathFind(36, run_state, facingRight);
    }
    else if (run_state == 2)
    {
      scoopRobot(run_state, scoop_count);
    }
    else if (run_state == 3)
    {
      compressRobot(90, 0.4,wall_dist_temp);
      dump(4500, 50, 0.2, wall_dist_temp, scoop_count); 
      end_or_search (search_width, search_length,facingRight,scoop_count, run_state);
    }
    else if (run_state == 4)
    {
      returning(36, run_state, facingRight, search_width, search_length);
    }
  }
  userInter(run_state);
  wait(5,seconds);
  Optical11.setLight(ledState::off);
  Brain.programStop();
  return EXIT_SUCCESS;
}
