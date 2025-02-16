#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;

// These are out of 127
using namespace ez;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(16.0, 0.0, 100.0);         // ez::fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(9.5, 0.0, 20);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.7, 0, 20, 15.0);     // Turn in place constants, old are 3.05, 0.05, 25.5, 15, negative kI are 2.5, -0.1, 10, 15
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 150_ms, 500_ms); // velocity original is 500 ms
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.odom_reset();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, false, true);
  chassis.pid_wait();
  /*for (int i=0;i<4;i++) {
    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
  }*/

  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  chassis.pid_turn_set(45_deg, 110);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(270_deg, 110);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();
  
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, 127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, 127);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// ORIENT YOURSELF TO THE FIELD:
// The field is oriented with the red alliance on the left and the blue alliance on the right.
// The center mobile goal should be lower.
// This orientation makes a cartesian plane with the origin at the center of the field.

void set_drive(double inches, int time, float minSpeed, float maxSpeed) {

    ez::pose c_pose = chassis.odom_pose_get();
    double trueAngle = c_pose.theta;
    chassis.pid_drive_set(inches, maxSpeed);
}

using namespace std;

int sign(bool isBlue) {
    return isBlue?1:-1;
}

/*
  goal: move as fast as possible in a straight line, ignoring accuracy
 */
void moveMax(double dist, int timeout) {
  // //these two variables used to track distance traveled
  // lemlib::Pose lastPose = chassis.getPose();
  // lemlib::Pose curPose = chassis.getPose();
  // double distTravelled = 0;
  // long startMillis = pros::millis(); // used to see how much time has elapsed
  // while (distTravelled < fabs(dist) && pros::millis() - startMillis < timeout) { // if robot has traveled far enough or time has gone over the limit
  //   chassis.tank(127 * sgn(dist), 127 * sgn(dist)); // move at max speed, account for backwards
  //   curPose = chassis.getPose();
  //   distTravelled += curPose.distance(lastPose); // find distance between positions and add to distance traveled
  //   lastPose = chassis.getPose();
  //   pros::delay(10);
  // }
  // setDrive(0, 0); // stop moving
}

void simpleRing(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, 90 * sgn);
	set_drive(-34 - 2, 2500, 0, 65);
	chassis.pid_wait_until(28);
	mogoClamp.toggle();
	chassis.pid_wait();
	//pros::delay(1000);
	pros::delay(500);
	intake.move_voltage(12000);
	pros::delay(700);
	chassis.pid_turn_set(0, 127);
	chassis.pid_wait();
	set_drive(21, 2000);
	chassis.pid_wait();
	pros::delay(1500);
	set_drive(-10, 1500);
	chassis.pid_wait();
	chassis.pid_turn_set(180, 127);
	chassis.pid_wait();
	set_drive(30, 2500, 0, 70);
	chassis.pid_wait();
  pros::delay(1000);
  intake.move(0);
	
}

void verySimpleMogo(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, 90 * sgn);
	set_drive(-34 - 2, 2500, 0, 65);
	chassis.pid_wait_until(28);
	mogoClamp.toggle();
	chassis.pid_wait();
	//pros::delay(1000);
	pros::delay(500);
	intake.move_voltage(12000);
	pros::delay(700);
	chassis.pid_turn_set(180, 127);
	chassis.pid_wait();
	set_drive(21, 2000);
	chassis.pid_wait();
	pros::delay(1500);
	set_drive(-10, 1500);
	chassis.pid_wait();
	chassis.pid_turn_set(0, 127);
	chassis.pid_wait();
	set_drive(30, 2500, 0, 70);
	chassis.pid_wait();
  pros::delay(1000);
  intake.move(0);
	
}

void simpleMogo(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, (33 + 1) * sgn);
  LBState = PROPPED;
  //LBRotation.set_position(4600);
  ladybrown2.set_zero_position(-46);
  ChangeLBState(FULLEXTENDED);
  pros::delay(650);
  set_drive(-15, 2000);
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set(90 * sgn, 127);
  chassis.pid_wait();
  set_drive(-23 - 11 + 3, 2000, 0, 70);
  chassis.pid_wait_until(25 + 1);
  mogoClamp.toggle();
  chassis.pid_wait();

	// set_drive(-34, 2000);
	// chassis.pid_wait_until(32 - 4);
	// mogoClamp.toggle();
	// chassis.pid_wait();
	intake.move_voltage(12000);
	//pros::delay(1000);
	chassis.pid_turn_set(180, 127);
	chassis.pid_wait();
	set_drive(21, 2000);
	chassis.pid_wait();
	pros::delay(500);
	set_drive(-20, 1500);
	chassis.pid_wait();
	//chassis.pid_turn_set(0);
  chassis.pid_turn_set((45 - 2) * sgn, 127);
	chassis.pid_wait();
	set_drive(50 - 20, 2000, 0, 60);
	chassis.pid_wait();
  intake.move(127);
  pros::delay(1000);
  set_drive(15, 1500);
  chassis.pid_wait();
  set_drive(-24 - 15, 1500, 70, 120);
  chassis.pid_wait();
  chassis.pid_turn_set(-45 * sgn, 127);
  chassis.pid_wait();
  // set_drive(5 + 1, 2000, 70, 120);
  // chassis.pid_wait();
  ChangeLBState(EXTENDED);
	
}

void newMogoRush(bool isBlue) {
  int sgn=isBlue?1:-1;
  chassis.odom_xyt_set(0, 0, -110 * sgn); // Set position
  LBState = PROPPED;
  //LBRotation.set_position(4600);
  ladybrown2.set_zero_position(-46);
  set_drive(37 + 2, 2500, 126, 127); // Move to first mogo
  chassis.pid_wait_until(11 + 1);
  ChangeLBState(ALMOSTFULLEXTENDED);
  chassis.pid_wait();
  // pros::delay(500);
  // chassis.pid_turn_set(-150 * sgn); // Turn mogo to disrupt
  // chassis.pid_wait();
  set_drive(-19.5 - 25 + 12, 1500, 120); // Move back
  chassis.pid_wait_until(30 - 14);
  ChangeLBState(FULLEXTENDED);
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set((-180 - 60 -10+3) * sgn, 127); // Turn to second mogo
  pros::delay(100);
  callLBReset();
  chassis.pid_wait();
  set_drive(-36.5 + 15, 2000); // Move to second mogo
  chassis.pid_wait_until(33 - 15);
  mogoClamp.toggle(); // Clamp second mogo
  chassis.pid_wait();
  set_drive(20, 2000, 65, 127); // Move back
  chassis.pid_wait();
  chassis.pid_turn_set((-140 + 5 + 5 + 5) * sgn, 127); // Turn to two stack
  chassis.pid_wait();
  intake.move(127); // Turn on intake
  set_drive(34.5 - 3 + 4, 2000); // Move to two stack
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_turn_set(isBlue?(106-10 + 5):-106, 127); // turn to corner
  chassis.pid_wait();
  rightDoinker.toggle();
  set_drive(40 - 5, 1500, 60, 127); // Move to corner
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(isBlue?-180:310, 127); // Turn to knock away corner rings
  chassis.pid_wait();
  rightDoinker.toggle();
  chassis.pid_turn_set(-260 * sgn, 127); // Turn to 2nd mogo
  chassis.pid_wait();
  set_drive(15 - 5, 2000, 60, 120); // Move forward
  chassis.pid_wait();
  set_drive(-10, 2000); // Move back
  chassis.pid_wait();
  // chassis.pid_turn_set(-300 * sgn); // Turn to 2nd ring
  // chassis.pid_wait();
  // set_drive(17 - 1, 2000); // Move to 2nd ring
  // chassis.pid_wait();
  //pros::delay(200);
  //chassis.pid_turn_set(300);
  // set_drive(-15 + 3, 2000); // Move back
  // chassis.pid_wait();
  chassis.pid_turn_set(-40 * sgn, 127); // Turn to ladder
  chassis.pid_wait();
  set_drive(46, 2000); // Move to ladder
  ChangeLBState(SEMIEXTENDED);
  chassis.pid_wait();
  ChangeLBState(EXTENDED);
}

void disruptRingRush(bool isBlue) {
  int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(55 * sgn,30,-(71 + 2.5) * sgn); //Starting Line 71
  setIntake(0);
  set_drive(38 + 1.5, 2500, 100, 127); //move and grab to rings 
  rushLeftPiston.toggle();
  rushRightPiston.toggle();
  chassis.pid_wait();
  set_drive(-12 + 1.5 - 3, 2000); // move back a little
  chassis.pid_wait();
  chassis.pid_turn_set(-97 * sgn, 127); // turn to go back
  chassis.pid_wait();
  set_drive(-4.5 - 3, 1500, 70, 127); // move back
  chassis.pid_wait_until(1 + 3);
  rushLeftPiston.toggle();
  rushRightPiston.toggle();
  chassis.pid_wait();
  // chassis.turnToPoint(24 * sgn, 21, 3000, {.forwards = false}); // turn to mogo
  //chassis.swingToHeading(55 * sgn, isBlue?lemlib::DriveSide::LEFT:lemlib::DriveSide::RIGHT, 3000);
  chassis.pid_turn_set((33 - 2) * sgn, 127); // turn to mogo
  chassis.pid_wait();
  set_drive(-18.5 - 5 + 3, 1500, 0, 65 + 10);
  chassis.pid_wait_until(14 + 1.5);
  mogoClamp.toggle(); // clamp mogo
  set_drive(-12 + 1.5, 1500, 70, 120); // move to mogo
  chassis.pid_wait();
  chassis.pid_turn_set((-8) * sgn, 127); // turn to intake rings
  //chassis.turnToPoint((24 - 4) * sgn, 60, 3000); // turn to intake rings
  chassis.pid_wait();
  pros::delay(100);
  set_drive(41 - 15, 2000, 0, 50 - 5); // move to intake rings
  chassis.pid_wait_until(8);
  intake.move(127); // start intake
  chassis.pid_wait();
  pros::delay(500);
  set_drive(13 - 2, 1500);
  chassis.pid_wait();
  
  pros::delay(500);
  chassis.pid_turn_set((135 - 1.5) * sgn, 127); // turn to preload
  chassis.pid_wait();
  set_drive(58.5 - 10, 2000, 70, 127); // move to preload
  chassis.pid_wait_until(40 - 30);
  ChangeLBState(PROPPED);
  chassis.pid_wait();
  pros::delay(900-300);
  chassis.pid_turn_set((125 - 5.5) * sgn, 127);
  chassis.pid_wait();
  intake.move(0);
  set_drive(11 - 2.25, 1500); // move to preload and score
  chassis.pid_wait();
  ChangeLBState(3);
  pros::delay(700 - 100);
  set_drive(-24.5 + 3 + 8, 2000, 70, 127); // move back
  chassis.pid_wait();
  ChangeLBState(REST);
  
}

void disruptRingRushBlue() {
  int sgn= 1;
	chassis.odom_xyt_set(55 * sgn,30,-(71 + 2.5) * sgn); //Starting Line 71
  setIntake(0);
  set_drive(38 + 0.25, 2500, 100, 127); //move and grab to rings 
  rushLeftPiston.toggle();
  rushRightPiston.toggle();
  chassis.pid_wait();
  set_drive(-12 + 1.5 - 3, 2000); // move back a little
  chassis.pid_wait();
  chassis.pid_turn_set(-97 * sgn, 127); // turn to go back
  chassis.pid_wait();
  set_drive(-4.5 - 3, 1500, 70, 127); // move back
  chassis.pid_wait_until(1 + 3);
  rushLeftPiston.toggle();
  rushRightPiston.toggle();
  chassis.pid_wait();
  // chassis.turnToPoint(24 * sgn, 21, 3000, {.forwards = false}); // turn to mogo
  //chassis.swingToHeading(55 * sgn, isBlue?lemlib::DriveSide::LEFT:lemlib::DriveSide::RIGHT, 3000);
  chassis.pid_turn_set((33 - 2) * sgn, 127); // turn to mogo
  chassis.pid_wait();
  set_drive(-18.5 - 5 + 3, 1500, 0, 65 + 10);
  chassis.pid_wait_until(14 + 1.5);
  mogoClamp.toggle(); // clamp mogo
  set_drive(-12 + 1.5, 1500, 70, 120); // move to mogo
  chassis.pid_wait();
  chassis.pid_turn_set((-8) * sgn, 127); // turn to intake rings
  //chassis.turnToPoint((24 - 4) * sgn, 60, 3000); // turn to intake rings
  chassis.pid_wait();
  pros::delay(100);
  set_drive(41 - 15, 2000, 0, 50 - 5); // move to intake rings
  chassis.pid_wait_until(8);
  intake.move(127); // start intake
  chassis.pid_wait();
  pros::delay(500);
  set_drive(13 - 2, 1500);
  chassis.pid_wait();
  
  pros::delay(500);
  chassis.pid_turn_set((135 - 1.5) * sgn, 127); // turn to preload
  chassis.pid_wait();
  set_drive(58.5 - 10, 2000, 70, 127); // move to preload
  chassis.pid_wait_until(40 - 30);
  ChangeLBState(PROPPED);
  chassis.pid_wait();
  pros::delay(900-300);
  chassis.pid_turn_set((125 - 5.5) * sgn, 127);
  chassis.pid_wait();
  intake.move(0);
  set_drive(11 - 2.25, 1500); // move to preload
  chassis.pid_wait();
  ChangeLBState(3);
  pros::delay(700 - 100);
  set_drive(-24.5 + 3 + 8, 2000, 70, 127); // move back
  chassis.pid_wait();
  ChangeLBState(REST);
  chassis.pid_turn_set((-110) * sgn, 127); // turn to ladder
  chassis.pid_wait();
  set_drive(26.5, 1500, 90, 100); // move forward
  chassis.pid_wait();
  ChangeLBState(PROPPED); 
}



void safeFourRing(bool isBlue) {


    // GETS CORNER
// Note: it does need to be this fast

	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, (33 + 1) * sgn);

// AWS

  LBState = PROPPED; // Prop LB for preload
  //LBRotation.set_position(4400);
  ladybrown2.set_zero_position(-46);
  ChangeLBState(EXTENDED); // Extend LB for AWS
  pros::delay(650 - 50);
  
  set_drive(-11 -1-1.5, 2000, 80); // move back from AWS
  chassis.pid_wait();
  ChangeLBState(REST); // retract ladybrown
  chassis.pid_turn_set(90 * sgn, 127);
  chassis.pid_wait();
  callLBReset();
  set_drive(-17, 2000, 70);
    chassis.pid_wait();
  set_drive(-12, 1500, 0, 45); // move slower
  chassis.pid_wait_until(-9);
  mogoClamp.toggle();
  chassis.pid_wait();
	intake.move_voltage(12000);
	chassis.pid_turn_set(180, 127); // turn to two stack
	chassis.pid_wait();
	// set_drive(27, 2000); // intake ring
	// chassis.pid_wait();
  	intake.move_voltage(0);
  	set_drive(19-3-1.5, 2000, 90); // intake ring
	chassis.pid_wait();
  intake.move_voltage(12000);
  	set_drive(7.5, 2000, 90); 
	chassis.pid_wait();
	pros::delay(300);
  //go to second two stack
  chassis.pid_turn_set(45 * sgn, 127); 
  chassis.pid_wait();
  set_drive(33 - 1, 2000, 50); 
  chassis.pid_wait();
  chassis.pid_turn_set(0 * sgn, 127); 

  // NOTE TO GAVIN: TURN OFF ANTI JAM HERE
  intakeUnstuckActivated = false;
  
  chassis.pid_wait();
  set_drive(26-3-8-1.5+1, 2000, 50); 
  intakeLift.toggle();
  chassis.pid_wait();
  intakeLift.toggle();
pros::delay(1000);
	// set_drive(-27, 1500, 0, 50); // move back

    //YOU CAN TURN IT BACK ON HERE
    intakeUnstuckActivated = true;


  set_drive(-15+3, 1500, 0, 50); // move back
	chassis.pid_wait();
  set_drive(6+2, 1500, 0, 50); // move 
  	chassis.pid_wait();
    set_drive(-18-2, 1500, 50); // move back
	chassis.pid_wait();
  chassis.pid_turn_set((-45 * sgn), 127);
  chassis.pid_wait();

  set_drive(25, 2000, 120); // move to ladder
  chassis.pid_wait_until(12);
  ChangeLBState(EXTENDED);
  
// 	set_drive(-27, 1500); // move back

//   chassis.pid_turn_set(100 * sgn, 100); // turn to corner
//   chassis.pid_wait();
//   set_drive(40); // move to corner
//   chassis.pid_wait();
//   pros::delay(500);
//   set_drive(-10); // move back
//   chassis.pid_wait();
//   set_drive(10); // intake 2nd ring
//   chassis.pid_wait();
//   set_drive(-10); // move back for real
//   chassis.pid_wait();

//   chassis.pid_turn_set(0 * sgn, 127); // turn to middle two stack
//   chassis.pid_wait();

//   set_drive(51, 2000, 50); // move to middle two stack
//   intakeLift.toggle();
//   chassis.pid_wait();
//   intakeLift.toggle();
// pros::delay(500);
  
// 	set_drive(-27, 1500); // move back
// 	chassis.pid_wait();
//   chassis.pid_turn_set((-30) * sgn, 127);
//   chassis.pid_wait();

  // set_drive(20, 2000, 120); // move to ladder
  // chassis.pid_wait();
  // ChangeLBState(EXTENDED);
	
}

void safeRingSide(bool isBlue) {
	int sgn=isBlue?1:-1;
	chassis.odom_xyt_set(0, 0, (146) * sgn);
  LBState = PROPPED;
  //LBRotation.set_position(4600);
// LB on aws
  ladybrown2.set_zero_position(-46);
  ChangeLBState(FULLEXTENDED);
  pros::delay(650);
  set_drive(-15, 2000);
  chassis.pid_wait();
  ChangeLBState(REST);

// Wanna try to get the 2 stack early bc we start next to it

// Get 2 stack next to aws
  intake.move(127); // start intake
  chassis.pid_turn_set(160, 120);
  chassis.pid_wait();
  intakeLift.toggle(); // lift intake
  set_drive(10 + 10); // move into ring
  chassis.pid_wait();
  intake.move(0); // stop intake to keep ring on intake
  intakeLift.toggle(); // lower intake on ring
  pros::delay(200 + 200);
  set_drive(-10 - 10); // move away from ring
  chassis.pid_wait();


// Getting mogo
  chassis.pid_turn_set(90 * sgn, 127);
  chassis.pid_wait();
  set_drive(-23 - 11 + 3, 2000, 0, 70);
  chassis.pid_wait_until(25 + 1);
  mogoClamp.toggle();
  chassis.pid_wait();

  // Getting middle 2 stacks
  chassis.pid_turn_set((-45 + 5) * sgn, 90); // Turn to first 2 stack
  chassis.pid_wait();
  // Intake ring
  intake.move(127);
  set_drive(21 + 2);
  chassis.pid_wait();
  // Turn to last 2 stack
  chassis.pid_turn_set((0) * sgn, 90);
  chassis.pid_wait();
  set_drive(20); // Intake 2 stack
  chassis.pid_wait();
  set_drive(-30 + 5); // Move back
  chassis.pid_wait();
  chassis.pid_turn_set((30+5) * sgn, 90); // Turn to final 2 stack on our quarter
  chassis.pid_wait();
  set_drive(20);
  chassis.pid_wait();
  chassis.pid_turn_set(65 * sgn, 90); // Turn to corner
  chassis.pid_wait();
  set_drive(45); // drive into corner
  chassis.pid_wait();
  chassis.pid_turn_set(45 * sgn, 90);
  chassis.pid_wait();
  set_drive(-10); // Move out of corner a bit
  chassis.pid_wait();

  /*
  chassis.pid_turn_set(180 * sgn, 90);
  chassis.pid_wait();
  intakeLift.toggle();
  set_drive(100);
  chassis.pid_wait_until(60);
  intakeLift.toggle();
  chassis.pid_wait();
  //Why do u guys need to do intake that ring like that
*/	
}

void MogoSide(bool isBlue) {
	chassis.odom_xyt_set(50_in,-36_in,114.3_deg); // Touching Red Ring Facing Mogo
	chassis.pid_drive_set(-(19.5 - 1), 127); // Move to Mogo
  chassis.pid_wait_until(17);
  mogoClamp.toggle(); // Clamp Mogo
  chassis.pid_wait();
	setIntake(127); // Score Ring
	chassis.pid_wait();
  chassis.pid_turn_set(307.75, 127); // Turn to Face Rings
  chassis.pid_wait();
  chassis.pid_drive_set((29.5 - 1), 127); // Reach Rings
  chassis.pid_wait_until(15);
  (isBlue?leftDoinker:rightDoinker).toggle(); // Toggle Doinker Rush Mech
  chassis.pid_wait();
  chassis.pid_turn_set(337.75_deg, 127); // Turn to Other Ring
  chassis.pid_wait();
  (!isBlue?leftDoinker:rightDoinker).toggle(); // Toggle Second Doinker Rush Mech
  chassis.pid_drive_set(-30, 127); // Move Back
  chassis.pid_wait();
  chassis.pid_turn_relative_set(30_deg, 127); // Turn To Ring
  chassis.pid_wait();
  chassis.pid_swing_set((!isBlue?ez::RIGHT_SWING:ez::LEFT_SWING), 180_deg, 90, 45); // Swing to Ring
  chassis.pid_wait();
  chassis.pid_turn_set({24,-48}, fwd, 127); // Turn to Ring 
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{24,-48}) - 1, 127); // Move to Ring
  chassis.pid_wait();
  chassis.pid_turn_set({66,66},fwd,127); // Turn to Corner
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{66,-66}) - 1, 127); // Move to Corner
  ChangeLBState(PROPPED); // Prop Lady Brown
  chassis.pid_wait();
  pros::delay(500);
  setIntake(0);
  chassis.pid_drive_set(-20,127); // Move Back
  chassis.pid_wait();
  chassis.pid_turn_set({6,-66},fwd,127); // Turn to Wall Stake
  chassis.pid_wait();
  chassis.pid_drive_set(ez::util::distance_to_point({chassis.odom_x_get(),chassis.odom_y_get()},{6,-66}) - 2, 127); // Move to Wall Stake
  chassis.pid_wait_quick();
  chassis.pid_turn_set({0,-72},fwd,127); // Correct to Wall Stake
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2,127); // Correct to Wall Stake
  chassis.pid_wait_quick();
  ChangeLBAuton(EXTENDED); // Score Ring on Wall Stake
  pros::delay(200);
  ChangeLBAuton(REST); // Bring LB Back
}
