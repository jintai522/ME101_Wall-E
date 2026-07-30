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
optical Optical11 = optical(PORT11);
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

// bump this string any time you want an easy way to confirm which version is loaded
const char* CODE_VERSION = "v11-infinite-search";

const double WHEEL_C = 200; //mm

const double ROTATE_KP = 1.8;
const double STRAIGHT_KP = 2;

const double SCOOPSPEED = 10;

const double COMPRESS_DEGREE = -4000;
const int COMPRESS_SPEED = 75;
const double COMPRESS_CURRENT_MAX = 0.9;

const double SHIFT_DISTANCE = 250; // mm, small lateral step between rows, not another full row
const int PATH_SPEED = 30;
const double TURN_ANGLE = 90;      // degrees; flip sign below if left/right come out reversed
const double GREEN_HUE_MIN = 75;     // degrees; widen/narrow this range while watching the live readout
const double GREEN_HUE_MAX = 160;
const double GREEN_BRIGHTNESS_MIN = 15; // percent; lower if tape never registers, raise if false triggers
const double RED_HUE_MIN = 340;      // degrees; red wraps around 0, so this is a two-sided range (>=340 or <=20)
const double RED_HUE_MAX = 20;
const double RED_BRIGHTNESS_MIN = 15; // percent; tune the same way as GREEN_BRIGHTNESS_MIN
const double CLEAR_DISTANCE = 150; // mm, blind drive after compressing so the sensor clears the spot
const int STATUS_REFRESH_INTERVAL = 10; // loop iterations between screen redraws

motor_group driveMotor(LeftMotor,RightMotor);

const double PI = 3.14159265358979;

// dead-reckoning odometry, relative to the robot's starting point/heading
double totalHeadingDeg = 0;
double posX = 0; // mm
double posY = 0; // mm

// the field's length varies box to box, so this is measured the first time a
// row is driven cleanly end-to-end (green tape to green tape), not hardcoded
double fieldLength = 0;
bool fieldLengthKnown = false;

void trackMove(double distanceTraveled)
{
  posX += distanceTraveled * sin(totalHeadingDeg * PI / 180.0);
  posY += distanceTraveled * cos(totalHeadingDeg * PI / 180.0);
}

double normalizeAngle(double angle)
{
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

double traveledDistance()
{
  return (fabs(LeftMotor.position(turns)*WHEEL_C)+fabs(RightMotor.position(turns)*WHEEL_C))/ 2.0;
}

void driveStep(int speed)
{
  double heading = BrainInertial.rotation(degrees);
  double error = 0 - heading; // target heading is 0 (straight)
  double correction = STRAIGHT_KP * error;

  LeftMotor.spin(forward, speed + correction, percent);
  RightMotor.spin(forward, speed - correction, percent);
}

void finishDrive()
{
  driveMotor.stop(brake);
  trackMove(traveledDistance());
}

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

  // motor momentum can carry the robot past the target after stop(brake) fires;
  // settle briefly, then apply small corrective nudges with a lower power floor
  // so it actually lands on the target instead of just reading close at the
  // instant the main loop's speed-5 minimum told it to stop
  wait(100, msec);
  int correctionAttempts = 0;
  error = targetAbsolute - BrainInertial.rotation(degrees);
  while (fabs(error) > 0.3 and correctionAttempts < 10)
  {
    double correctionPower = (error > 0) ? 3 : -3;
    if (correctionPower > 0)
    {
      LeftMotor.spin(forward, correctionPower, percent);
      RightMotor.spin(reverse, correctionPower, percent);
    }
    else
    {
      LeftMotor.spin(reverse, -correctionPower, percent);
      RightMotor.spin(forward, -correctionPower, percent);
    }
    wait(30, msec);
    driveMotor.stop(brake);
    wait(50, msec);
    error = targetAbsolute - BrainInertial.rotation(degrees);
    correctionAttempts++;
  }

  totalHeadingDeg = normalizeAngle(totalHeadingDeg + targetAngle);
}

void showStatusLine1(const char* text)
{
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearLine(1);
  Brain.Screen.print("%s", text);
}

void showSensorReadout(double hue, double brightness, color col)
{
  static int sampleCount = 0;

  // throttle the readout so the screen redraw doesn't slow down sensing
  if (sampleCount % STATUS_REFRESH_INTERVAL == 0)
  {
    Brain.Screen.setCursor(2,1);
    Brain.Screen.clearLine(2);
    Brain.Screen.print("hue %.0f bright %.0f", hue, brightness);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.clearLine(3);
    Brain.Screen.print("color: %s", convertColorToString(col));
  }
  sampleCount++;
}

bool scoopDetect(double min, double max, int & run_state)
{
  if(Distance2.objectDistance(mm) < max and Distance2.objectDistance(mm) > min)
  {
    showStatusLine1("Object Detected");
    run_state = 2;
    return true;
  }
  else
  {
    return false;
  }
}

void scoopermove (double scooperSpeed, int & run_state)
  //go back a bit
  //bring down scooper
  //go foward
  //bring up scooper
{
  LeftMotor.spin(reverse,15,percent);
  RightMotor.spin(reverse,15,percent);
  wait(2.8,seconds);
  LeftMotor.stop();
  RightMotor.stop();

  wait(0.5,seconds);

  Scooper8.spin(forward,scooperSpeed,percent);
  wait(4, seconds);
  Scooper8.stop();

  wait(0.5,seconds);

  LeftMotor.spin(forward,100,percent);
  RightMotor.spin(forward,100,percent);
  wait(1,seconds);
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




void straightDrive(double distance, int speed,int & run_state)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (traveledDistance() < distance and !(scoopDetect(10,30,run_state)))
  {
    driveStep(speed);
  }
  finishDrive();
}

void userInter(int & run_state)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  if (run_state == 1) Brain.Screen.print("Searching");
  else if (run_state == 2) Brain.Screen.print("Scooping");
  else if (run_state == 3) Brain.Screen.print("Compressing");
  else if (run_state == 4) Brain.Screen.print("Heading to disposal box");
  else if (run_state == 0) Brain.Screen.print("Finished");
}

bool seesGreenTape()
{
  double hue = Optical11.hue();
  double brightness = Optical11.brightness();
  bool isGreen = (hue >= GREEN_HUE_MIN and hue <= GREEN_HUE_MAX and brightness > GREEN_BRIGHTNESS_MIN);

  showSensorReadout(hue, brightness, Optical11.color());
  if (isGreen) showStatusLine1("Green Tape Detected");

  return isGreen;
}

bool seesRedTape()
{
  double hue = Optical11.hue();
  double brightness = Optical11.brightness();
  bool isRed = (hue >= RED_HUE_MIN or hue <= RED_HUE_MAX) and brightness > RED_BRIGHTNESS_MIN;

  showSensorReadout(hue, brightness, Optical11.color());
  if (isRed) showStatusLine1("Red Tape Detected");

  return isRed;
}

void driveUntilGreen(int speed, int & run_state)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (!seesGreenTape() and !(scoopDetect(10,30,run_state)))
  {
    driveStep(speed);
  }
  double distance = traveledDistance();
  finishDrive();

  // capture the field's length the first time a row completes cleanly (reached
  // green tape, wasn't cut short by an object) -- this is how big the box
  // actually is this run, since it varies and can't be hardcoded
  if (run_state != 2 and !fieldLengthKnown)
  {
    fieldLength = distance;
    fieldLengthKnown = true;
  }
}

void driveUntilRed(int speed)
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (!seesRedTape())
  {
    driveStep(speed);
  }
  finishDrive();
}

void driveBlind(double distance, int speed)
  // drives a fixed distance with no scoopDetect check, used to clear the
  // distance sensor away from a just-compacted spot before re-arming detection
{
  BrainInertial.resetRotation();
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (traveledDistance() < distance)
  {
    driveStep(speed);
  }
  finishDrive();
}

void pathFind(int & run_state)
{
  static int row = 0; // persists across calls so we resume where we left off after a scoop/compress

  for (;; row++) // search indefinitely -- only a detected object breaks this loop
  {
    driveUntilGreen(PATH_SPEED, run_state);
    if (run_state == 2) return; // object detected mid-row; scoopermove/compress will run, then we resume this row

    int turnSign = (row % 2 == 0) ? -1 : 1; // alternate direction each row transition (L-L, R-R, L-L, ...)
    rotateRobot(turnSign * TURN_ANGLE, PATH_SPEED);
    straightDrive(SHIFT_DISTANCE, PATH_SPEED, run_state);
    if (run_state == 2) return;
    rotateRobot(turnSign * TURN_ANGLE, PATH_SPEED);
  }
}

void spinCompressorTo(double degree, int speed, double current_max, directionType dir)
{
  MotorCompress9.resetPosition();
  MotorCompress9.spin(dir, speed, percent);
  while (fabs(MotorCompress9.position(degrees)) <= fabs(degree) and MotorCompress9.current(amp) <= current_max)
  {}
  MotorCompress9.stop(brake);
}

void compress(double degree, int speed, double current_max, int & run_state)
{
  spinCompressorTo(degree, speed, current_max, reverse);
  wait(1, seconds);
  spinCompressorTo(degree, speed, current_max, forward);

  driveBlind(CLEAR_DISTANCE, PATH_SPEED); // move past the compacted spot so it doesn't immediately re-trigger scoopDetect

  run_state = 4; // head to the disposal box
}

void dumpTrash()
{
  Scooper8.spin(forward, SCOOPSPEED, percent); // bring the scooper down
  wait(2, seconds);
  Scooper8.stop();

  spinCompressorTo(COMPRESS_DEGREE, COMPRESS_SPEED, COMPRESS_CURRENT_MAX, reverse); // push all the trash out
  wait(1, seconds);
  spinCompressorTo(COMPRESS_DEGREE, COMPRESS_SPEED, COMPRESS_CURRENT_MAX, forward); // retract back
}

void goToDisposalBox(int & run_state)
{
  // the box sits on the same starting column (X=0), halfway down the field's length
  double targetX = 0;
  double targetY = fieldLength / 2.0;

  double dx = targetX - posX;
  double dy = targetY - posY;
  double distance = sqrt(dx*dx + dy*dy);
  double targetHeading = atan2(dx, dy) * 180.0 / PI;

  rotateRobot(normalizeAngle(targetHeading - totalHeadingDeg), PATH_SPEED);
  driveBlind(distance, PATH_SPEED);

  rotateRobot(normalizeAngle(90 - totalHeadingDeg), PATH_SPEED); // absolute right, relative to the very first starting heading

  driveUntilRed(PATH_SPEED);

  dumpTrash();

  run_state = 0;
}

int main()
{
  configureAllSensors();
  int run_state = 1;
  /*
    0 close program
    1 run pattern
    2 run scooping
    3 run compress
    4 go to disposal box and dump
  */

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Code %s", CODE_VERSION);
  Brain.Screen.setCursor(2,1);
  Brain.Screen.print("Press to start");

  while (!Brain.buttonCheck.pressing())
  {}
  while (Brain.buttonCheck.pressing())
  {}

  while (run_state != 0)
  {
    userInter(run_state);

    if (run_state == 1)
    {
      pathFind(run_state);
    }
    else if (run_state == 2)
    {
      scoopermove(SCOOPSPEED, run_state);
    }
    else if (run_state == 3)
    {
      compress(COMPRESS_DEGREE, COMPRESS_SPEED, COMPRESS_CURRENT_MAX, run_state);
    }
    else if (run_state == 4)
    {
      goToDisposalBox(run_state);
    }
  }

  userInter(run_state);
  Optical11.setLight(ledState::off);
  Brain.programStop();
  return EXIT_SUCCESS;
}
