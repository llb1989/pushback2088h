#include "main.h"
#include "pros/llemu.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-16, 9, -5}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup rightMotors({8, -15, 12}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

// top right is 15
// bottom back right is 8
// bottom front right is 17

// top left is 9
// bottom back left is 5
// bottom front left is 16 

pros::adi::Button autonselectbutton('C');


//intake mototro
pros::Motor intmotor1(-20); // first stage // 20 
pros::Motor intmotor2(-2); // middle roller // 2
pros::Motor intmotor3(-4); // top // 41

// Inertial Sensor on port 19
pros::Imu imu(14);

pros::Rotation rotation(17);
lemlib::TrackingWheel horizontal_tracking_wheel(&rotation, lemlib::Omniwheel::NEW_275, -4.66);

pros::adi::Pneumatics littlewill('A', false);
pros::adi::Pneumatics chickenstars('B', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 25 holes?
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(11, // proportional gain (kP) 5.58
                                            0, // integral gain (kI)
                                            6, // derivative gain (kD) 19.05
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.85, // proportional gain (kP) 
                                             0, // integral gain (kI) 
                                             13.15, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(
    // &vertical, // vertical tracking wheel
                            nullptr,
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
