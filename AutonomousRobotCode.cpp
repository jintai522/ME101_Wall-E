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
// v18: compress() now pushes inward until current crosses COMPRESS_PUSH_CURRENT_THRESHOLD
// (adopted from Jintai's current-sensing compress) instead of a fixed degree target;
// goToDisposalBox() now uses a fixed turn/tape-crossing maneuver (adopted from Jintai's
// returning() idea) instead of odometry-vector navigation.
// v19: driveUntilGreen() now also checks for blue tape mid-search (seesBlueTape(),
// adopted from Jintai's BLUE_HUE_MIN/MAX) via disposalSignalDetect() -- seeing blue sets
// run_state = 4 to head back, same as finishing a compress. The disposal zone itself
// is still marked by red tape (driveUntilRed/seesRedTape, unchanged).
// v20: SLOWDOWN_START_FRACTION dropped from 0.80 to 0.70 -- eases back to 1x speed
// earlier in the row so there's more room to settle before the green tape.
// v21: compress()'s push-inward step was spinning past where the extend step started
// because spinCompressorUntilCurrent() had no degree cap -- now uses spinCompressorTo()
// again (degree AND current threshold) so it retracts/pushes the same |degree| (4000)
// as the extend step, not an unbounded spin waiting for a current spike.
// v22: full debug pass. Fixed pathFind()/goToDisposalBox() double-turning the heading
// when an object interrupts them mid-maneuver and they get resumed (a static guard now
// prevents the turn from re-running). Removed dead odometry code (posX/posY/trackMove/PI)
// left over from the goToDisposalBox rewrite -- it ran trig on every finishDrive() call
// for values nothing read anymore.
const char* CODE_VERSION = "v22-debug-pass";

const double WHEEL_C = 200; //mm

const double ROTATE_KP = 1.8;
const double STRAIGHT_KP = 2;

const double SCOOPSPEED = 10;

const double COMPRESS_DEGREE = -4000;
const int COMPRESS_SPEED = 75;
const double COMPRESS_CURRENT_MAX = 0.9;
const double COMPRESS_PUSH_CURRENT_THRESHOLD = 0.4; // amps; push-inward stops once current crosses this (Jintai's idea) instead of a fixed degree

const double SHIFT_DISTANCE = 250; // mm, small lateral step between rows, not another full row
const int PATH_SPEED = 45; // 30 * 1.5
const double TURN_ANGLE = 90;      // degrees; flip sign below if left/right come out reversed
const double SCOOP_DETECT_MIN = 10;  // mm; tune both of these while watching real detections
const double SCOOP_DETECT_MAX = 60;  // mm; 150 was catching the ground since the sensor sits slightly angled down
const int STOP_RAMP_STEP = 10;    // percent power dropped per ramp step when easing to a stop
const int STOP_RAMP_DELAY = 30;   // msec between ramp steps
const double FAST_MULTIPLIER = 2.0; // speed multiplier once the row length is known
const double SLOWDOWN_START_FRACTION = 0.70; // slow back to 1x once this fraction of the known length is covered
const double GREEN_HUE_MIN = 75;     // degrees; widen/narrow this range while watching the live readout
const double GREEN_HUE_MAX = 160;
const double GREEN_BRIGHTNESS_MIN = 15; // percent; lower if tape never registers, raise if false triggers
const double RED_HUE_MIN = 340;      // degrees; red wraps around 0, so this is a two-sided range (>=340 or <=20)
const double RED_HUE_MAX = 20;
const double RED_BRIGHTNESS_MIN = 15; // percent; tune the same way as GREEN_BRIGHTNESS_MIN
const double BLUE_HUE_MIN = 190;     // degrees; the blue return-signal tape (Jintai's idea)
const double BLUE_HUE_MAX = 359;
const double BLUE_BRIGHTNESS_MIN = 15; // percent; tune the same way as GREEN_BRIGHTNESS_MIN
const double CLEAR_DISTANCE = 150; // mm, blind drive after compressing so the sensor clears the spot
const int STATUS_REFRESH_INTERVAL = 10; // loop iterations between screen redraws

motor_group driveMotor(LeftMotor,RightMotor);

// heading reference, relative to the robot's starting orientation (never reset mid-run)
double totalHeadingDeg = 0;

// the field's length varies box to box, so this is measured the first time a
// row is driven cleanly end-to-end (green tape to green tape), not hardcoded
double fieldLength = 0;
bool fieldLengthKnown = false;

int lastTurnSign = -1; // direction of the most recent pathFind row-shift turn, needed again
                        // for the second turn when a resumed call skips recomputing it

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

void driveStep(double speed)
  // speed can be negative to drive in reverse -- still heading-corrected either way
{
  // correct toward totalHeadingDeg (the true expected heading for the whole run)
  // instead of "0" -- the inertial sensor is never reset mid-run anymore, so
  // this is a real absolute reference instead of "whatever I was facing when
  // this leg started," which is what let misalignment quietly compound before.
  double heading = BrainInertial.rotation(degrees);
  double error = totalHeadingDeg - heading;
  double correction = STRAIGHT_KP * error;

  double leftPower = speed + correction;
  double rightPower = speed - correction;

  LeftMotor.spin((leftPower >= 0) ? forward : reverse, fabs(leftPower), percent);
  RightMotor.spin((rightPower >= 0) ? forward : reverse, fabs(rightPower), percent);
}

void driveForDuration(double seconds, double speed)
  // heading-corrected driving for a fixed duration (not a fixed distance) --
  // used by scoopermove so the scoop-approach stays aligned instead of
  // drifting off heading like the old raw equal-percentage commands did
{
  int steps = (int)(seconds * 1000 / STOP_RAMP_DELAY);
  for (int i = 0; i < steps; i++)
  {
    driveStep(speed);
    wait(STOP_RAMP_DELAY, msec);
  }
  driveMotor.stop(brake);
}

void finishDrive(double fromSpeed)
  // eases to a stop instead of slamming the brake -- an abrupt full-brake
  // stop (especially from the faster 2x search speed) was what caused the
  // wobble/misalignment right as an object was detected
{
  double p = fromSpeed;
  while (fabs(p) > STOP_RAMP_STEP)
  {
    p = (p > 0) ? p - STOP_RAMP_STEP : p + STOP_RAMP_STEP;
    driveStep(p);
    wait(STOP_RAMP_DELAY, msec);
  }
  driveMotor.stop(brake);
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

  // kept unwrapped (not normalized) so it stays on the same raw scale as
  // BrainInertial.rotation(degrees), which is also never reset mid-run and
  // therefore never wraps either -- normalizeAngle is only applied to
  // differences of this value elsewhere, never to the value itself
  totalHeadingDeg = totalHeadingDeg + targetAngle;
}

void snapHeadingToGrid()
  // re-anchors heading drift against the tape itself: corrects to the nearest
  // 90-degree multiple, since the field's rows are expected to run exactly
  // perpendicular to the tape lines. Run this every time tape is reached so
  // small errors get corrected instead of silently carrying into the next row.
{
  double nearestGrid = round(totalHeadingDeg / 90.0) * 90.0;
  double correction = normalizeAngle(nearestGrid - totalHeadingDeg);

  if (fabs(correction) > 0.5)
  {
    rotateRobot(correction, PATH_SPEED);
  }
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
  // every drivetrain move below is heading-corrected (driveForDuration), so a
  // misaligned approach doesn't get worse during the scoop and cause a miss
{
  driveForDuration(2.8, -15);

  wait(0.5,seconds);

  Scooper8.spin(forward,scooperSpeed,percent);
  wait(4, seconds);
  Scooper8.stop();

  wait(0.5,seconds);

  driveForDuration(1, 100);
  driveForDuration(0.5, 67);
  driveForDuration(0.5, 23);

  Scooper8.spin(reverse,47,percent);
  wait(1, seconds);
  Scooper8.stop();

  run_state = 3;
}




void straightDrive(double distance, int speed,int & run_state)
{
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (traveledDistance() < distance and !(scoopDetect(SCOOP_DETECT_MIN,SCOOP_DETECT_MAX,run_state)))
  {
    driveStep(speed);
  }
  finishDrive(speed);
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

bool seesBlueTape()
  // a signal encountered while searching -- seeing it means "head back to the
  // disposal zone now," adopted from Jintai's seesColor(BLUE_HUE_MIN, BLUE_HUE_MAX, ...)
  // check inside his driveUntilGreen(). Red (seesRedTape) is what actually marks the
  // dump zone itself -- blue is just the trigger to go look for it.
{
  double hue = Optical11.hue();
  double brightness = Optical11.brightness();
  bool isBlue = (hue >= BLUE_HUE_MIN and hue <= BLUE_HUE_MAX) and brightness > BLUE_BRIGHTNESS_MIN;

  showSensorReadout(hue, brightness, Optical11.color());
  if (isBlue) showStatusLine1("Blue Tape Detected");

  return isBlue;
}

bool disposalSignalDetect(int & run_state)
  // checked during searching -- if blue tape is seen, stop searching and head to the
  // disposal zone (goToDisposalBox will navigate to the red tape from there)
{
  if (seesBlueTape())
  {
    run_state = 4;
    return true;
  }
  return false;
}

void driveUntilGreen(int speed, int & run_state)
{
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  // first pass: fieldLength isn't known yet, so just drive at base speed to measure it.
  // every pass after that: drive at FAST_MULTIPLIER speed, then ease back to base speed
  // once SLOWDOWN_START_FRACTION of the known length is covered, so it doesn't blow past the tape.
  double slowdownStart = fieldLength * SLOWDOWN_START_FRACTION;
  int stepSpeed = speed;

  while (!seesGreenTape() and !(scoopDetect(SCOOP_DETECT_MIN,SCOOP_DETECT_MAX,run_state)) and !(disposalSignalDetect(run_state)))
  {
    stepSpeed = speed;
    if (fieldLengthKnown and traveledDistance() < slowdownStart)
    {
      stepSpeed = speed * FAST_MULTIPLIER;
    }
    driveStep(stepSpeed);
  }
  double distance = traveledDistance();
  finishDrive(stepSpeed);

  if (run_state != 2) // stopped because of tape, not an object -- a real tape crossing
  {
    snapHeadingToGrid();

    // capture the field's length the first time a row completes cleanly (reached
    // green tape, wasn't cut short by an object) -- this is how big the box
    // actually is this run, since it varies and can't be hardcoded
    if (!fieldLengthKnown)
    {
      fieldLength = distance;
      fieldLengthKnown = true;
    }
  }
}

void driveUntilRed(int speed)
{
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (!seesRedTape())
  {
    driveStep(speed);
  }
  finishDrive(speed);
}

void driveBlind(double distance, int speed)
  // drives a fixed distance with no scoopDetect check, used to clear the
  // distance sensor away from a just-compacted spot before re-arming detection
{
  LeftMotor.resetPosition();
  RightMotor.resetPosition();

  while (traveledDistance() < distance)
  {
    driveStep(speed);
  }
  finishDrive(speed);
}

void pathFind(int & run_state)
{
  static int row = 0; // persists across calls so we resume where we left off after a scoop/compress
  static bool turnedForShift = false; // true once this row's first turn has run, so resuming after an
                                       // object interrupts the shift drive doesn't repeat that turn

  for (;; row++) // search indefinitely -- only a detected object breaks this loop
  {
    if (!turnedForShift)
    {
      driveUntilGreen(PATH_SPEED, run_state);
      if (run_state == 2) return; // object detected mid-row; scoopermove/compress will run, then we resume this row
      if (run_state == 4) return; // blue return-signal seen; goToDisposalBox will head to the red disposal tape

      lastTurnSign = (row % 2 == 0) ? -1 : 1; // alternate direction each row transition (L-L, R-R, L-L, ...)
      rotateRobot(lastTurnSign * TURN_ANGLE, PATH_SPEED);
      turnedForShift = true;
    }

    straightDrive(SHIFT_DISTANCE, PATH_SPEED, run_state);
    if (run_state == 2) return; // object detected mid-shift; resumes here next time, skipping the turn above

    rotateRobot(lastTurnSign * TURN_ANGLE, PATH_SPEED);
    turnedForShift = false;
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
  // push-inward step stops at whichever comes first: the current threshold (Jintai's
  // idea -- resistance sensed) or the same degree magnitude as the extend step, so the
  // compressor can't keep spinning past where it started if current never spikes
{
  spinCompressorTo(degree, speed, current_max, reverse); // extend/open the compressor arm over the trash
  wait(1, seconds);
  spinCompressorTo(degree, speed, COMPRESS_PUSH_CURRENT_THRESHOLD, forward); // push inward, capped at the same |degree|

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
  // fixed-maneuver return (adopted from Jintai's returning() idea) instead of the
  // odometry-vector approach: turn back toward the start column, cross the nearest
  // row-boundary green tape, turn again, then drive along that edge until the red
  // disposal tape is found. No posX/posY/fieldLength math needed here -- compress()
  // already drove past the compacted spot, so no extra backup step is needed either.
{
  static bool turnedToStartColumn = false; // guards the first turn so resuming after an
                                            // object interrupts this leg doesn't re-turn

  if (!turnedToStartColumn)
  {
    rotateRobot(-TURN_ANGLE, PATH_SPEED); // toward the start column
    turnedToStartColumn = true;
  }

  driveUntilGreen(PATH_SPEED, run_state); // reach the row-boundary tape
  if (run_state == 2) return; // object detected; resumes here next time, skipping the turn above

  driveBlind(SHIFT_DISTANCE, PATH_SPEED); // cross fully over it, onto the edge column
  rotateRobot(TURN_ANGLE, PATH_SPEED); // face down the edge column toward the disposal marker
  driveUntilRed(PATH_SPEED);

  dumpTrash();

  turnedToStartColumn = false;
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
