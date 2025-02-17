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

    pros::delay(200);
    intake.move(-127);
    pros::delay(450);
    intake.move(0);

    set_drive(-19.5, 1500, 0, 70); // move away from alliance stake
    chassis.pid_wait_until(-16);


    /////////////////////////// FIRST MOGO ///////////////////////////
    /////////////////////////// FIRST MOGO ///////////////////////////
    

    mogoClamp.toggle(); // clamp mogo
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    //pros::delay(100);

    setIntake(127); // score on alliance stake
    chassis.pid_turn_set(90, 90); // turn to two rings
    chassis.pid_wait();

    chassis.pid_drive_set(17 - 1, 2500); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(116.5, 90); // turn to corner ring stack
    chassis.pid_wait();
    chassis.pid_drive_set(79 - 25, 115); // move to next two rings in corner 2 stack
    chassis.pid_wait_until(45);
    ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_turn_set(109 - 10, 90); // turn to go back
    chassis.pid_wait();
    intake.move_voltage(12000);
    chassis.pid_drive_set(-43 + 17.5 - 0.5, 110); // move back a bit
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
    //ColorLoopActive = true;
    chassis.pid_drive_set(15, 80);
    chassis.pid_wait_until(5);
    intake.move(127);
    //startColorUntil(1);
    chassis.pid_wait_until(13 - 2);
    //chassis.pid_wait();

    //setIntake(0);

    ChangeLBState(EXTENDED); // extend ladybrown
    pros::delay(400);
    // chassis.pid_turn_set(180, 127); // correct angle
    // chassis.pid_wait();
    // set_drive(-8); // go back a bit
    // ChangeLBState(PROPPED); // retract ladybrown to propepd
    // chassis.pid_wait();

    // set_drive(8);
    // chassis.pid_wait_until(1);
    // intake.move(127); 
    // chassis.pid_wait();
    
    // pros::delay(200);
    // ChangeLBState(EXTENDED); // extend ladybrown
    // pros::delay(350 + 50);
    // intake.move(127);

    set_drive(-12.8 - 0.5); // go back a bit
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown

    chassis.pid_turn_set(-89.9 + 2, 90); // turn to two rings
    setIntake(127);
    chassis.pid_wait();
    intake.move_voltage(12000);

    // collecting 3 rings

    callLBReset();

    set_drive(58.5, 3000, 0, 70); 
    chassis.pid_wait_until(29);
    chassis.pid_speed_max_set(55);
    //set_drive(28 + 2, 3000, 0, 70); // intake rings slowly
    chassis.pid_wait();
    setIntake(127);
    pros::delay(800 - 300);
    chassis.pid_turn_set(140, 90); // turn to last ring before corner
    chassis.pid_wait();
    setIntake(127);
    set_drive(13 - 0.5, 1500, 75, 120); // collect last bottom-left ring
    chassis.pid_wait();
    chassis.pid_turn_set(65, 90 - 35); // turn to corner
    chassis.pid_wait();
    setIntake(127);

    set_drive(-14 - 1, 750); // move to corner
    chassis.pid_wait();
    //pros::delay(300 + 100);
    mogoClamp.toggle(); // unclamp mogo
    setIntake(0);

    set_drive(6.75 + 3.5, 1500, 60, 120); // move out of corner
    chassis.pid_wait();
    chassis.pid_turn_set(180 + 2, 90);
    chassis.pid_wait();
    setIntake(0);

    set_drive(-80 - 1); // move to mogo
    chassis.pid_wait_until(-40);
    chassis.pid_speed_max_set(70);
    chassis.pid_wait_until(-75);
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

    chassis.pid_drive_set(20 + 1, 2500); // move to first ring
    chassis.pid_wait();
    chassis.pid_turn_set(63 + 1, 90); // turn to second ring
    chassis.pid_wait();
    chassis.pid_drive_set(74 + 2, 2500); // move to second and third ring
    chassis.pid_wait_until(35);
    chassis.pid_speed_max_set(40);
    chassis.pid_wait_until(71);
    ChangeLBState(PROPPED);
    chassis.pid_wait();
    chassis.pid_turn_set(70, 90); // turn to go back
    chassis.pid_wait();
    // Making only 1 turn
    // chassis.pid_turn_set(60, 90); // turn to next two rings
    // chassis.pid_wait();
    // chassis.pid_drive_set(50 + 3, 2500); // move to next two rings
    // chassis.pid_wait();
    // ChangeLBState(PROPPED); // prop up ladybrown
    chassis.pid_drive_set(-45 + 1.5, 110); // move back a bit
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);
    chassis.pid_turn_set(0, 90); // turn to wall stake
    chassis.pid_wait();
    
    //chassis.follow(skills2_txt, 15, 4000); // pure pursuit 3 rings
    //chassis.pid_wait();
    // set_drive(-45, 1500, 80); // move back
    // chassis.pid_wait();

    // chassis.pid_turn_set(0.699233+1 - 2, 69.8696 + 3, 127);
    // chassis.pid_wait();
    ColorLoopActive = true;
    startColorUntil(1);
    chassis.pid_drive_set(18, 100);
    chassis.pid_wait_until(5);
    intake.move(70);
    // chassis.pid_odom_set({0.699233+1 - 2, 69.86966 + 3});
    // chassis.pid_wait();
    chassis.pid_wait_until(14);
    ChangeLBState(EXTENDED); // extend ladybrown
    chassis.pid_wait();
    pros::delay(300 - 150);
    chassis.pid_turn_set(0, 127); // correct angle
    chassis.pid_wait();
    set_drive(-10, 700); // move back
    chassis.pid_wait_until(-5);
    ChangeLBState(PROPPED); // retract ladybrown
    chassis.pid_wait();
    pros::delay(100);
    set_drive(12, 700); // move to wall stake
    chassis.pid_wait_until(4);
    stopColorUntilFunction();
    setIntake(127);
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(EXTENDED);
    pros::delay(300);

    set_drive(-15.5 + 2, 3000); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    setIntake(127);
    chassis.pid_turn_set(-90 + 1, 127); // turn to three rings
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
    set_drive(11.5 + 2, 1500, 75, 120); // move to ring before corner
    chassis.pid_wait();

    chassis.pid_turn_set(112 + 5, 100 - 45); // turn to corner
    chassis.pid_wait();

    set_drive(-16 + 3, 1000, 70, 120); // back INto corner
    chassis.pid_wait();
    callLBReset();
    //pros::delay(300);
    intake.move(0);
    mogoClamp.toggle(); // unclamp mogo

    ColorLoopActive = true;

    // TRANSITION PERIOD
    // TRANSITION PERIOD

    chassis.pid_turn_set(135 + 3, 90); // turn to intake ring
    chassis.pid_wait();
    startColorUntil(1); // stop first red ring at top
    set_drive(80, 3000, 80, 127); // go to intake ring
    chassis.pid_wait_until(50);
    intake.move(90);
    chassis.pid_wait();
    chassis.pid_turn_set(45, 90); // turn to second ring
    chassis.pid_wait();
    set_drive(35.5, 2000, 0, 75); // go to second ring
    chassis.pid_wait_until(30 - 4);
    ChangeLBState(PROPPED);
    chassis.pid_wait();
    stopColorUntilFunction();
    intake.move_voltage(12000);
    chassis.pid_turn_set(-45 + 7, 90); // turn to mogo
    chassis.pid_wait();
    setIntake(0);
    ChangeLBState(SEMIEXTENDED);
    pros::delay(200);
    intake.move(100);
    startColorUntil(1);
    chassis.pid_drive_set(-35, 2000); // move to mogo
    chassis.pid_wait_until(-15);
    chassis.pid_speed_max_set(70);
    chassis.pid_wait_until(-32);
    mogoClamp.toggle();


    /////////////////////////// THIRD MOGO ///////////////////////////
    /////////////////////////// THIRD MOGO ///////////////////////////

    chassis.pid_wait();
    stopColorUntilFunction();
    intake.move(127);
    chassis.pid_turn_set(90, 90); // turn to AWS
    chassis.pid_wait();
    chassis.pid_drive_set(15, 2500, 0, 70); // move to AWS
    chassis.pid_wait();
    ChangeLBState(FULLEXTENDED); // extend ladybrown
    pros::delay(50);
    chassis.pid_drive_set(-16, 1500); // move back
    chassis.pid_wait();
    ChangeLBState(REST); // retract ladybrown
    chassis.pid_turn_set(-135, 90); // turn to get ring outside of ladder
    chassis.pid_wait();
    set_drive(28 + 2); // move to ring outside ladder
    chassis.pid_wait();
    chassis.pid_turn_set(135, 90); // turn to intake first two stack 
    chassis.pid_wait();
    intake.move_voltage(12000);
    set_drive(35 - 3); // go to intake first two stack
    chassis.pid_wait();
    chassis.pid_turn_set(100, 90); // turn to intake second two stack
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(15, 110); // go to intake second two stack
    chassis.pid_wait();
    chassis.pid_turn_set(135, 90); // turn to intake third two stack
    chassis.pid_wait();
    // chassis.pid_drive_set(-35, 110); // go back
    // chassis.pid_wait();
    // chassis.pid_turn_set(110, 90); // turn to intake third two stack
    // chassis.pid_wait();
    rightDoinker.toggle();
    set_drive(27 - 12); // go to intake third two stack
    chassis.pid_wait();
    chassis.pid_turn_set(215, 90); // turn to intake third two stack
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::e_swing::RIGHT_SWING, -60, 120); // swing to corner
    chassis.pid_wait();
    set_drive(-15); // move back into corner
    chassis.pid_wait();
    mogoClamp.toggle(); // release mogo
    rightDoinker.toggle();
    set_drive(35 - 20);
    chassis.pid_wait();
    chassis.pid_turn_set(18, 90); // turn to intake third two stack
    chassis.pid_wait();
    set_drive(100, 3000, 0, 100);
    chassis.pid_wait();
    ChangeLBState(SEMIEXTENDED);
    chassis.pid_turn_set(45, 90); // turn to hang on ladder
    chassis.pid_wait_quick_chain();
    set_drive(-65);
    chassis.pid_wait_until(-40);
    chassis.pid_speed_max_set(70);



    

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