#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

void skills() {
    chassis.odom_xyt_set(-59.15, -10.6, -45); //Wait is angle, -50.5? // starts at middle of red alliance line
    LBState = PROPPED; // Prop LB for preload
    ladybrown2.set_zero_position(-46);
    ChangeLBState(EXTENDED); // Extend LB for AWS
    pros::delay(200);
    intake.move(-127); // Band Release
    pros::delay(400);
    intake.move(0);
    ChangeLBState(PROPPED);
    chassis.pid_drive_set(-10,127);
    chassis.pid_wait();
    mogoClamp.toggle();
    pros::delay(100);
    chassis.pid_turn_set({-24_in,-24_in},fwd,127);
    chassis.pid_wait();
    chassis.pid_odom_set({-24_in,-24_in},127);
    chassis.pid_wait();
}