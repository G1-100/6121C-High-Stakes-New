#include "pros/motors.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

void skills() {
    chassis.odom_look_ahead_set(10);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    chassis.odom_xyt_set(-60.5, -13, (-46)); // starts at middle of red alliance line
    
}

void skillsMacro() {
   
}