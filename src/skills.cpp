#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

void skills() {
    chassis.odom_xyt_set(-60.5, -13, (-49 + 3)); // starts at middle of red alliance line
    //pros::Task lb_task(LBLoop);
    LBState = EXTENDED;
    LBRotation.set_position(STOP1);
    ladybrown2.set_zero_position(-46);
    ladybrown2.set_zero_position(-46);
    ChangeLBState(FULLEXTENDED);
    intakeUnstuckActivated = true;
    ColorLoopActive = false;

    pros::delay(650);

    set_drive(-19.5 - 1, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(16 + 1);


    /////////////////////////// FIRST MOGO ///////////////////////////
    /////////////////////////// FIRST MOGO ///////////////////////////
    

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    //pros::delay(100);

    setIntake(127); // score on alliance stake
    chassis.pid_turn_set(95, 90); // turn to two rings
    chassis.pid_wait();

    chassis.pid_drive_set(20 - 3, 2500); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(116, 90); // turn to next two rings
    chassis.pid_wait();
    chassis.pid_drive_set(73 + 5, 2500); // move to next two rings
    chassis.pid_wait();
    intake.move_voltage(12000);
    ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_turn_set(109, 90); // turn to next two rings
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_drive_set(-48 - 0.5, 110); // move back a bit
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);
    chassis.pid_turn_set(180, 90); // turn to wall stake
    chassis.pid_wait();


    //chassis.follow(skills1_txt, 15, 2500); // pure pursuit two rings
    //chassis.pid_wait_until(30);
    //callLBReset();
    //chassis.pid_wait();
    //chassis.pid_turn_set(-1.2156, -54.6709, 127); // turn to wall stake ring
    //chassis.pid_wait();
    //ChangeLBState(PROPPED); // prop up ladybrown

    //chassis.pid_odom_set({-1.2156, -54.6709});
    ColorLoopActive = true;
    chassis.pid_drive_set(15, 100);
    chassis.pid_wait_until(5);
    intake.move(127);
    startColorUntil(1);
    chassis.pid_wait_until(12);

    setIntake(0);

    ChangeLBState(EXTENDED); // extend ladybrown
    pros::delay(350 + 50);
    // chassis.pid_turn_set(180, 127); // correct angle
    // chassis.pid_wait();
    set_drive(-8); // go back a bit
    ChangeLBState(PROPPED); // retract ladybrown to propepd
    chassis.pid_wait();

    set_drive(8);
    chassis.pid_wait_until(1);
    intake.move(127);
    chassis.pid_wait();
    
    pros::delay(350);
    intake.move(0);

    ChangeLBState(EXTENDED); // extend ladybrown
    pros::delay(350 + 50);
    intake.move(127);

    set_drive(-12.8 - .5); // go back a bit
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    chassis.pid_turn_set(-89.9 + 2, 90); // turn to two rings
    setIntake(127);
    chassis.pid_wait();
    intake.move_voltage(12000);

    // collecting 3 rings

    callLBReset();

    set_drive(56, 3000, 0, 60 + 20); 
    chassis.pid_wait_until(30 - 1);
    set_drive(28 + 2, 3000, 0, 70); // intake rings slowly
    chassis.pid_wait();
    setIntake(127);
    pros::delay(800 - 300);
    chassis.pid_turn_set(140, 90); // turn to last ring before corner
    chassis.pid_wait_until(10);
    chassis.pid_wait();
    setIntake(127);
    set_drive(14.5 - 1.5, 1500, 75, 120); // collect last bottom-left ring
    chassis.pid_wait();
    chassis.pid_turn_set(65, 90); // turn to corner
    chassis.pid_wait();
    setIntake(127);

    set_drive(-14 - 1, 750); // move to corner
    chassis.pid_wait();
    pros::delay(300 + 100);
    mogoClamp.toggle(); // unclamp mogo
    setIntake(0);

    set_drive(6.75 + 3, 1500, 60, 120); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(180 + 2, 90);
    chassis.pid_wait();
    setIntake(0);

    set_drive(-75 - 2, 2500 + 250, 80, 120); // move to mogo
    chassis.pid_wait_until(45);
    chassis.pid_speed_max_set(60);
    chassis.pid_wait_until(70);
    // set_drive(-21 + 5, 1500, 0, 60);
    // chassis.pid_wait_until(20.5 - 5);
    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();


    /////////////////////////// SECOND MOGO ///////////////////////////
    /////////////////////////// SECOND MOGO ///////////////////////////

    // set_drive(3, 500, 40); // move back
    // chassis.pid_wait();
    chassis.pid_turn_set(90, 127); // turn to pure pursuit two stacks
    chassis.pid_wait();
    setIntake(127);
    callLBReset(); // reset ladybrown

    chassis.pid_drive_set(20, 2500); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(60, 90); // turn to next two rings
    chassis.pid_wait();
    chassis.pid_drive_set(50 + 3, 2500); // move to next two rings
    chassis.pid_wait();
    ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_drive_set(-45 - 3, 110); // move back a bit
    chassis.pid_wait();
    chassis.pid_turn_set(0, 90); // turn to wall stake
    chassis.pid_wait();
    
    //chassis.follow(skills2_txt, 15, 4000); // pure pursuit 3 rings
    //chassis.pid_wait();
    // set_drive(-45, 1500, 80); // move back
    // chassis.pid_wait();

    chassis.pid_turn_set(0.699233+1 - 2, 69.8696 + 3, 127);
    chassis.pid_wait();
    ColorLoopActive = true;
    startColorUntil(1);
    chassis.pid_odom_set({0.699233+1 - 2, 69.86966 + 3});
    chassis.pid_wait();
    setIntake(0);

    ChangeLBState(EXTENDED); // extend ladybrown
    pros::delay(300);
    chassis.pid_turn_set(0, 127); // correct angle
    chassis.pid_wait();
    set_drive(-10, 700); // move back
    ChangeLBState(PROPPED); // retract ladybrown
    chassis.pid_wait();
    set_drive(12, 700); // move to wall stake
    chassis.pid_wait_until(1);
    setIntake(127);
    chassis.pid_wait();
    ChangeLBState(EXTENDED);

    set_drive(-15.5 + 2.5, 3000); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    setIntake(127);
    chassis.pid_turn_set(-93 + 1.5 - 3.5, 127); // turn to three rings
    chassis.pid_wait();
    intake.move_voltage(12000);

    // collecting 3 rings

    set_drive(57, 2000, 0, 60 + 15);
    callLBReset();
    chassis.pid_wait_until(30 - 1);
    //chassis.cancelMotion();
    set_drive(28.25, 2500, 0, 30 + 30); // intake rings slowly
    chassis.pid_wait();
    pros::delay(800);
    chassis.pid_turn_set(40, 127); // turn to last ring before corner
    chassis.pid_wait();
    set_drive(10 + 1.5, 1500, 75, 120); // move to ring before corner
    chassis.pid_wait();

    chassis.pid_turn_set(107 + 5, 127); // turn to corner
    chassis.pid_wait();

    set_drive(-13 - 3, 1000, 70, 120); // back INto corner
    chassis.pid_wait();
    callLBReset();
    pros::delay(300);
    intake.move(0);
    mogoClamp.toggle(); // unclamp mogo

    ColorLoopActive = true;

/*

    chassis.pid_turn_set(135, 127); // turn to intake ring
    chassis.pid_wait();
    startColorUntil(1); // stop first red ring at top
    set_drive(110, 3000, 80, 127); // go to intake ring
    chassis.pid_wait_until(50);
    intake.move_voltage(12000);
    chassis.pid_wait_until(100);
    ChangeLBState(PROPPED);
    chassis.pid_wait();
    intake.move(127);
    chassis.pid_turn_set(-135, 127); // turn to third mogo
    startColorUntil(1); // stop for 2nd red ring
    chassis.pid_wait();
    set_drive(-35.5, 2000, 0, 75); // go to third mogo
    chassis.pid_wait_until(33);
    mogoClamp.toggle();


// Stopping before third half for tuning    

    /////////////////////////// THIRD MOGO ///////////////////////////
    /////////////////////////// THIRD MOGO ///////////////////////////


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

void skillsMacro() {
    chassis.odom_xyt_set(-60.5, -13, (-49)); // starts at middle of red alliance line
    setIntake(127); // score on alliance stake
    pros::Task lb_task(LBLoop);
    LBState = EXTENDED;
    //LBRotation.set_position(4600);
    ladybrown1.set_zero_position(-46);
    ChangeLBState(FULLEXTENDED);

    pros::delay(650);

    set_drive(-16 - 1.5, 1500); // move away from alliance stake
    chassis.pid_wait_until(12);
    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    //LBRetract();
    ChangeLBState(REST); // retract ladybrown
    master.rumble("."); // short rumble to notify driver

*/ 

}