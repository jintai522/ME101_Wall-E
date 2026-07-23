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
motor Scooper8 = motor(PORT8, false);


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
const double KP = 1.8;
void rotateRobot(double targetAngle, int speed)
{
  double error = 676; // change if needed
  double startAngle = BrainInertial.rotation(degrees);
  double targetAbsolute = startAngle + targetAngle;
  while (fabs(error) > 0.3) // get at 0.3 degree of target
  {
    double currentAngle = BrainInertial.rotation(degrees);
    error = targetAbsolute - currentAngle;

    double power = KP * error;
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

double scooperTime(double scooperdegree, double scooperspeed)
{
  double result = scooperdegree / (scooperspeed / 100 * 120 / 60 * 360);
  return result;
}

void scoopermove (double scooperDegree, double scooperSpeed,int & run_state)
  //go back a bit
  //bring down scooper
  //go foward
  //bring up scooper
{
  LeftMotor.spin(reverse,50,percent);
  RightMotor.spin(reverse,50,percent);
  wait(1.5,seconds);
  LeftMotor.stop();
  RightMotor.stop();

  wait(0.5,seconds);

  Scooper8.spin(forward,scooperSpeed,percent);
  wait(4, seconds);
  Scooper8.stop();

  wait(0.5,seconds);

  LeftMotor.spin(forward,100,percent);
  RightMotor.spin(forward,100,percent);
  wait(1.4,seconds);
   LeftMotor.spin(forward,67,percent);
  RightMotor.spin(forward,67,percent);
  wait(0.5,seconds);
  LeftMotor.spin(forward,23,percent);
  RightMotor.spin(forward,23,percent);
  wait(0.5,seconds);
  LeftMotor.stop();
  RightMotor.stop();

  Scooper8.spin(reverse,47,percent);
  wait(1, seconds);
  Scooper8.stop();

  run_state = 3; 
  
}




const double STRAIGHT_KP = 2;
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
    double heading = BrainInertial.rotation(degrees);
    double error = 0 - heading; // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;

    LeftMotor.spin(forward, speed + correction, percent);
    RightMotor.spin(forward, speed - correction, percent);
  }
  driveMotor.stop(brake);
}

void userInter(int & run_state)
{
  Brain.Screen.clearScreen();
  //out put whats its doing during the state
}

void pathDrive(double distance, int speed)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (((fabs(LeftMotor.position(turns)*WHEEL_C)+fabs(RightMotor.position(turns)*WHEEL_C))/ 2.0 < distance))
  {
    double heading = BrainInertial.rotation(degrees);
    double error = 0 - heading; // target heading is 0 (straight)

    double correction = STRAIGHT_KP * error;

    LeftMotor.spin(forward, speed + correction, percent);
    RightMotor.spin(forward, speed - correction, percent);
  }
  driveMotor.stop(brake);
}

const double FIELD_LENGTH = 1000;  // mm, field length (direction each row drives)
const double FIELD_WIDTH = 1000;   // mm, field width (direction rows shift across)
const int NUM_ROWS = 5;            // reasonable zigzag density for the field size
const double SHIFT_DISTANCE = FIELD_WIDTH / (NUM_ROWS - 1); // small step, not another full row
const int PATH_SPEED = 30;
const double TURN_ANGLE = 90;      // degrees; flip sign below if left/right come out reversed

void pathFind()
{
  int turnSign = -1; // -1 = left, 1 = right; flip if turns come out reversed

  for (int row = 0; row < NUM_ROWS; row++)
  {
    pathDrive(FIELD_LENGTH, PATH_SPEED);

    if (row < NUM_ROWS - 1) // no shift/turn needed after the last row
    {
      rotateRobot(turnSign * TURN_ANGLE, PATH_SPEED);
      pathDrive(SHIFT_DISTANCE, PATH_SPEED);
      rotateRobot(turnSign * TURN_ANGLE, PATH_SPEED);
      turnSign = -turnSign; // alternate direction each row transition (L-L, R-R, L-L, ...)
    }
  }
}

void compress(double degree, int speed, double current_max, int & run_state)
{
  MotorCompress9.resetPosition();
  MotorCompress9.spin(forward, speed, percent);
  while (fabs(MotorCompress9.position(degrees)) <= fabs(degree) and MotorCompress9.current(amp) <= current_max)
  {}
  MotorCompress9.stop(brake);

  wait(1, seconds);

  MotorCompress9.resetPosition();
  MotorCompress9.spin(reverse, speed, percent);
  while (fabs(MotorCompress9.position(degrees)) <= fabs(degree) and MotorCompress9.current(amp) <= current_max)
  {}
  MotorCompress9.stop(brake);

  run_state = 4;
}

  double scooperDegree = 67;
  double scooperSpeed = 10;

  const double COMPRESS_DEGREE = -4000;
  const int COMPRESS_SPEED = 75;
  const double COMPRESS_CURRENT_MAX = 0.9;

int main()
{
  configureAllSensors();

  while (!Brain.buttonCheck.pressing())
  {}
  while (Brain.buttonCheck.pressing())
  {}

  pathFind();

  Optical8.setLight(ledState::off);
  Brain.programStop();
  return EXIT_SUCCESS;
}
