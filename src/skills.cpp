#include "pros/motors.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

void skills() {
    chassis.odom_xyt_set(-60.5, -13, (-46)); // starts at middle of red alliance line
    //pros::Task lb_task(LBLoop);
    LBState = EXTENDED;
    LBRotation.set_position(STOP1);
    ladybrown2.set_zero_position(-46);
    ChangeLBState(FULLEXTENDED);
    intakeUnstuckActivated = true;
    ColorLoopActive = false;

    pros::delay(200);
    intake.move(-127);
    pros::delay(300);
    intake.move(0);

    set_drive(-19.5, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(-16);


    /////////////////////////// FIRST MOGO ///////////////////////////
    /////////////////////////// FIRST MOGO ///////////////////////////
    

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    setIntake(127);
    intake.move(127);
    chassis.pid_turn_set(90, 90); // turn to two rings
    chassis.pid_wait();

    chassis.pid_drive_set(17 - 1, 110); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(116.5, 90); // turn to corner ring stack
    chassis.pid_wait();
    chassis.pid_drive_set(79 - 25 + 2, 115); // move to next two rings in corner 2 stack
    chassis.pid_wait_until(45 - 20);
    ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_turn_set(109 - 10, 90); // turn to go back
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_drive_set(-43 + 14 + 1.75, 100); // move back a bit
    chassis.pid_wait();
    chassis.pid_turn_set(180, 90); // turn to wall stake
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);

    // chassis.pid_drive_set(14, 80, false, false);
    // chassis.pid_wait_until(5);
    // intake.move(127);
    // //chassis.pid_wait_until(14);
    // chassis.pid_wait();


    setDrive(110, 110);
    pros::delay(150);
    intake.move(127);
    pros::delay(400);

    ChangeLBState(EXTENDED); // extend ladybrown
    pros::delay(400);
    
    set_drive(-12.8 + 0.25); // go back a bit
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    chassis.pid_turn_set(-89.9 + 2, 90); // turn to two rings
    setIntake(127);
    chassis.pid_wait();
    intake.move_voltage(12000);

    // collecting 3 rings

    callLBReset();

    set_drive(58.5, 3000, 0, 70);  // cap the max speed at 70
    chassis.pid_wait_until(29);
    chassis.pid_speed_max_set(50); // intake rings slower
    chassis.pid_wait();
    setIntake(127);
    chassis.pid_turn_set(140 - 10 + 10, 60); // turn to last ring before corner
    chassis.pid_wait_quick_chain();
    setIntake(127);
    set_drive(13.5 + 2 - 2, 1500, 75, 120); // collect last bottom-left ring
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(75 - 5, 90 - 35); // turn to corner
    chassis.pid_wait();
    setIntake(127);

    set_drive(-16 - 1, 750); // move to corner
    chassis.pid_wait();
    //pros::delay(500);
    mogoClamp.toggle(); // unclamp mogo
    setIntake(0);

    set_drive(10.25 - 1.25, 1500, 60, 120); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(181, 90);
    chassis.pid_wait();
    setIntake(0);

    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    double wallDist = (rightAlignmentSensor.get_distance() * 0.0393701) * abs(cos(chassis.odom_theta_get() * M_PI / 180));
    std::cout << "WALL DIST: " << wallDist << std::endl;
    if (leftAlignmentSensor.get_confidence() > 10) {
        chassis.odom_x_set(-72 + wallDist + 2.5255);
    }

    std::cout << "NEW POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    set_drive(-79 - 4 + 1.5, 110); // move to mogo
    chassis.pid_wait_until(-45);
    chassis.pid_odom_set({{-48, 20}, rev, 60});
    //chassis.pid_wait_until_point({-48, -10});
    //chassis.pid_speed_max_set(70 - 15);
    chassis.pid_wait_until(-78 + 45);
    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();

    

    /////////////////////////// SECOND MOGO ///////////////////////////
    /////////////////////////// SECOND MOGO ///////////////////////////

    chassis.pid_turn_set(90, 127); // turn to pure pursuit two stacks
    chassis.pid_wait();
    setIntake(127);
    callLBReset(); // reset ladybrown

    chassis.pid_drive_set(20 + 1, 110); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(64.5 - 1, 90); // turn to second ring
    chassis.pid_wait();
    chassis.pid_drive_set(78 + 1, 110); // move to second and third ring
    chassis.pid_wait_until(35);
    chassis.pid_speed_max_set(45);
    chassis.pid_wait_until(71);
    ChangeLBState(PROPPED);
    chassis.pid_wait();
    chassis.pid_turn_set(70, 90); // turn to go back
    chassis.pid_wait();
    chassis.pid_drive_set(-46.5 + 1.5, 110); // move back a bit
    chassis.pid_wait();
    setIntake(0);
    chassis.pid_turn_set(0, 90); // turn to wall stake
    chassis.pid_wait();
    ChangeLBState(SEMIEXTENDED);

    ColorLoopActive = true;
    startColorUntil(1);
    chassis.pid_drive_set(17, 110, false, false);
    chassis.pid_wait_until(5);
    intake.move(70); // intake slower to stop inside intake

    //chassis.pid_wait_until(17);
    chassis.pid_wait();
    setDrive(110, 110);
    ChangeLBState(EXTENDED); // extend ladybrown
    chassis.pid_wait();
    pros::delay(300 - 150);
    set_drive(-10 - 5, 700); // move back
    chassis.pid_wait_until(-11);
    ChangeLBState(PROPPED); // retract ladybrown for 2nd extension
    chassis.pid_wait();
    pros::delay(200);
    set_drive(12 + 5); // move to wall stake
    stopColorUntilFunction();
    chassis.pid_wait_until(4);
    setIntake(127); // intake into ladybrown
    chassis.pid_wait();
    setDrive(80, 80);
    setIntake(0);
    ChangeLBState(EXTENDED);
    pros::delay(300);

    set_drive(-15.5 + 2, 3000); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    setIntake(127);
    chassis.pid_turn_set(-90 + 1, 127); // turn to three rings
    chassis.pid_wait();

    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    wallDist = (rightAlignmentSensor.get_distance() * 0.0393701) * abs(sin(chassis.odom_theta_get() * M_PI / 180));
    std::cout << "WALL DIST: " << wallDist << std::endl;
    if (rightAlignmentSensor.get_confidence() > 10) {
        chassis.odom_y_set(72 - wallDist - 4.5);
    }

    intake.move_voltage(12000);
    stopColorUntilFunction();

    // collecting 3 rings

    //set_drive(57, 2000, 0, 75 - 5);
    chassis.pid_odom_set({{-55.13, 48}, fwd, 70});
    callLBReset();
    chassis.pid_wait_until(29 - 3);
    //chassis.cancelMotion();
    set_drive(28.3 + 3 + 1, 2500, 0, 40 + 15); // intake rings slowly
    chassis.pid_wait();
    pros::delay(200);
    std::cout << "POSITION: " << chassis.odom_x_get() << " " << chassis.odom_y_get() << " " << chassis.odom_theta_get() << std::endl;
    chassis.pid_turn_set(40, 50); // turn to last ring before corner
    chassis.pid_wait_quick_chain();
    set_drive(13.5 - 2, 1500, 75, 120); // move to ring before corner
    chassis.pid_wait_quick_chain();
    //pros::delay(100);
    chassis.pid_turn_set(112 + 5, 100 - 45); // turn to corner
    chassis.pid_wait_quick_chain();

    set_drive(-16 + 3, 1000, 70, 120); // back into corner
    chassis.pid_wait();
    callLBReset();

    intake.move(-127);
    pros::delay(300 - 100);
    intake.move(0);
    mogoClamp.toggle(); // unclamp mogo

    ColorLoopActive = true;

    // TRANSITION PERIOD
    // TRANSITION PERIOD

    chassis.pid_turn_set(135 + 3, 90); // turn to intake ring
    chassis.pid_wait();
    startColorUntil(1); // stop first red ring at top
    set_drive(80+3, 3000, 80, 127); // go to intake ring
    chassis.pid_wait_until(75);
    intake.move(110);
    chassis.pid_wait();
    // set_drive(-3,3000,80,110);
    // chassis.pid_wait();
    chassis.pid_turn_set(45, 90); // turn to second ring
    chassis.pid_wait();
    set_drive(35.5, 2000, 0, 75); // go to second ring
    chassis.pid_wait_until(22 - 1.5);
    ChangeLBState(PROPPED);
    stopColorUntilFunction();
    chassis.pid_wait_until(27 + 2);
    intake.move_voltage(12000);
    chassis.pid_wait();
    pros::delay(300 - 50);
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);
    pros::delay(200);
    startColorUntil(1);
    intake.move(110);
    chassis.pid_drive_set(5, 110); // move in a little more
    chassis.pid_wait();
    chassis.pid_drive_set(-5, 110); // move back
    chassis.pid_wait();

    chassis.pid_turn_set(-45 + 7 - 2, 90); // turn to mogo
    chassis.pid_wait();
    chassis.pid_drive_set(-34 + 1.5, 2000); // move to mogo
    chassis.pid_wait_until(-14);
    chassis.pid_speed_max_set(70);
    chassis.pid_wait_until(-32.5);
    mogoClamp.toggle();


    /////////////////////////// THIRD MOGO ///////////////////////////
    /////////////////////////// THIRD MOGO ///////////////////////////

    chassis.pid_wait();
    stopColorUntilFunction();
    chassis.pid_turn_set(90, 90); // turn to AWS
    chassis.pid_wait();
    intake.move(127);
    chassis.pid_drive_set(15, 90, false, false); // move to AWS
    chassis.pid_wait();
    ChangeLBState(FULLEXTENDED); // extend ladybrown
    chassis.pid_drive_set(-16, 1500); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    chassis.pid_turn_set(-135, 90); // turn to get ring outside of ladder
    chassis.pid_wait();
    set_drive(28 + 2 + 3); // move to ring outside ladder
    chassis.pid_wait();
    chassis.pid_turn_set(136 - 5, 60); // turn to intake first two stack 
    chassis.pid_wait();
    intake.move_voltage(12000);
    set_drive(32 - 3); // go to intake first two stack
    chassis.pid_wait();
    chassis.pid_turn_set(100, 90 - 15); // turn to intake second two stack
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(15 - 3, 110); // go to intake second two stack
    chassis.pid_wait();
    startColorUntil(1);
    chassis.pid_turn_set(225, 90 - 10); // turn to intake third two stack
    chassis.pid_wait();
    stopColorUntilFunction();
    intake.move(127);
    // chassis.pid_drive_set(-35, 110); // go back
    // chassis.pid_wait();
    // chassis.pid_turn_set(110, 90); // turn to intake third two stack
    // chassis.pid_wait();
    set_drive(27 - 12); // go to intake third two stack
    chassis.pid_wait();
    set_drive(-5); // move back
    chassis.pid_wait();
    
    //rightDoinker.toggle();
    leftDoinker.toggle();
    // chassis.pid_turn_set(120, 127);
    // chassis.pid_wait();
    // set_drive(6, 110); // go to corner
    // chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(130, 100); // turn to corner

    //chassis.pid_swing_set(ez::e_swing::RIGHT_SWING, -60, 120); // swing to corner
    chassis.pid_wait();
    set_drive(6, 110); // move to corner
    chassis.pid_wait();

    chassis.pid_turn_behavior_set(ez::counterclockwise);
    chassis.pid_turn_set(-60, 127); // turn to corner
    chassis.pid_wait();

    chassis.pid_turn_behavior_set(ez::shortest);
    //rightDoinker.toggle();
    mogoClamp.toggle(); // release mogo
    set_drive(-12 - 30); // move back into corner
    chassis.pid_wait_until(-15 + 2);
    setIntake(0);
    //rightDoinker.toggle();
    leftDoinker.toggle();
    set_drive(14 + 1);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(20 - 5, 90); // turn to last mogo
    chassis.pid_wait();

    /////////////////////////// FOURTH MOGO ///////////////////////////
    /////////////////////////// FOURTH MOGO ///////////////////////////

    chassis.slew_drive_constants_set(1_in, 127);

    chassis.pid_drive_set(1000, 127, true, true);
    chassis.pid_wait_until(80);
    setDrive(127, 127);
    long start = pros::millis();
    while (pros::millis() - start < 800 + 1000 - 1000) { // manual timeout of 1 second
        pros::delay(20);
    }
    //ChangeLBState(SEMIEXTENDED);
    chassis.pid_turn_set(45, 90); // turn to hang on ladder
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-65 - 5, 127, false);
    chassis.pid_wait_until(-50);
    chassis.pid_speed_max_set(70 + 10);


/*
    chassis.pid_wait();
    pros::delay(200);
    //ChangeLBState(EXTENDED); // extend ladybrown a little
    setIntake(127);
    chassis.pid_turn_set(90 - 2, 127); // turn to AWS
    chassis.pid_wait();
    set_drive(4.5 + 10, 1500 - 500, 0, 70); // move to AWS
    chassis.pid_wait();
    set_drive(-8, 1500); // move back
    setIntake(0);
    ChangeLBState(FULLEXTENDED); // extend ladybrown
    pros::delay(700);
    set_drive(-5 + 0.25, 2000, 60, 120); // move back
    setIntake(127);

    chassis.pid_turn_set(-45, 127); // turn to pure pursuit 3 rings
    chassis.pid_wait();
    //chassis.follow(skills5_txt, 13, 3500); // pure pursuit 3 rings top right
    chassis.pid_wait();
    set_drive(-16, 1000, 70, 127); // move back after done
    chassis.pid_wait();
    chassis.pid_turn_set(180, 127); // turn to bottom left rings
    chassis.pid_wait();
    set_drive(140, 2500, 120, 127); // go to bottom left rings
    chassis.pid_wait();

    chassis.pid_turn_set(90, 127); // turn to intake second stack
    chassis.pid_wait();
    set_drive(10, 1500, 60, 120); // intake second stack
    chassis.pid_wait();
    //pros::delay(500);
    chassis.pid_turn_set(-145 - 5, 127); // turn to third two stack
    chassis.pid_wait();
    set_drive(13, 2000); // intake third two stack
    chassis.pid_wait();
    rightDoinker.toggle();
    pros::delay(500);

    
    chassis.pid_turn_set(-58 + 5, 127); // turn to corner, sweep with rightDoinker
    //chassis.swingToHeading(-55 + 20, lemlib::DriveSide::LEFT, 1000); // turn to corner
    chassis.pid_wait();
    //pros::delay(400);
    mogoClamp.toggle(); // unclamp mogo
    set_drive(-6 - 5, 1200, 100, 120); // move to corner
    chassis.pid_wait();
    setIntake(0);
    setIntake(0);
    //pros::delay(200);
    chassis.pid_turn_set(-45, 127); // turn to corner
    chassis.pid_wait();
    set_drive(20 - 3, 2000, 90, 127); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(20, 127); // turn to 4th mogo


    /////////////////////////// FOURTH MOGO ///////////////////////////
    /////////////////////////// FOURTH MOGO ///////////////////////////


    chassis.pid_wait();
    callLBReset();
    set_drive(108 - 4, 3200, 120, 127); // push other mogo to corner
    chassis.pid_wait();
    set_drive(-10, 1500, 0, 60); // move back

}

*/
}

void skillsMacro() {
    chassis.odom_xyt_set(-60.5, -13, (-49 + 3)); // starts at middle of red alliance line
    //pros::Task lb_task(LBLoop);
    LBState = EXTENDED;
    ladybrown2.set_zero_position(-46);
    ChangeLBState(FULLEXTENDED);
    intakeUnstuckActivated = true;
    ColorLoopActive = true;

    pros::delay(200);
    intake.move(-127);
    pros::delay(400);
    intake.move(0);

    set_drive(-19.5, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(-16);

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    master.rumble("."); // short rumble to notify driver

}

void recalibrateUsingDistance() {
    // double theta = chassis.odom_theta_get() % 360.0;
    // if (theta < 0) {
    //     theta += 360.0;
    // }
    // if (theta > 270) {

    // }
}