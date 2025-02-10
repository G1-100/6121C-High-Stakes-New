#include "main.h"
#include "pros/motors.h"
#include "lemlib/api.hpp"



//MOTORS

//drive
pros::MotorGroup left_side_motors({5, -6, 9}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup right_side_motors({2, -1, 7}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

//intake
pros::Motor intake(9, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// ladybrown wall stake mech
pros::Motor ladybrown1(-11, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);
pros::Motor ladybrown2(-20, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);


//pistons
pros::adi::Pneumatics intakeLift('H', false);
pros::adi::Pneumatics doinker('H', false); 
pros::adi::Pneumatics mogoClamp('D', false);

// inertial
pros::Imu IMU(4);

bool allianceColorBlue = true;

// rotational sensor
pros::Rotation horizTracking(12);
pros::Rotation vertTracking(19);
pros::Rotation LBRotation(7);

pros::Optical optical(20);

extern pros::Distance autoClampSensor(1);


//CONTROLLERS
pros::Controller master(pros::E_CONTROLLER_MASTER);



	lemlib::Drivetrain drivetrain(
		&left_side_motors, 
		&right_side_motors, 
		12.5, // track width
		lemlib::Omniwheel::NEW_275, // wheel diameter
		450, // rpm
        8 // horizontal drift
	);

    lemlib::ControllerSettings lateral_controller(
        11, // proportional gain (kP)
        0, // integral gain (kI)
        45 + 10, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        0 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings angular_controller(  
        7, // proportional gain (kP) safe: 3
        0, // integral gain (kI)
        55, // derivative gain (kD) 20
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        2, // large error range, in inches
        500, // large error range timeout, in milliseconds
        0 // maximum acceleration (slew)
    );

    lemlib::TrackingWheel horizTrackingWheel(&horizTracking, lemlib::Omniwheel::NEW_275, -5.75);
    lemlib::TrackingWheel vertTrackingWheel(&vertTracking, lemlib::Omniwheel::NEW_275, -2.5);

	lemlib::OdomSensors sensors(nullptr,//&vertTrackingWheel, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
        nullptr,//&vertTrackingWheel, // horizontal tracking wheel 1
        nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
        &IMU // inertial sensor
    );

	// lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

    // Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    343);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels
