#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstddef>


// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups   
pros::MotorGroup leftMotors({-17, 9, -18}, pros::MotorGearset::blue);   
pros::MotorGroup rightMotors({5, -16, 14 }, pros::MotorGearset::blue);  

// top right is 1 - acc 20 real
// bottom back right is 6 - acc 16 real 69
// bottom front right is 12 - acc 14 real

// top left is 15 - acc 17 real
// bottom back left is 8 - acc 11 real
// bottom front left is 14 - acc 18 real

pros::adi::Button autonselectbutton('C');


//intake mototro
pros::Motor intmotor1(-8); // first stage // 4
pros::Motor intmotor3(-15); // top // 19

// Inertial Sensor on port 19
pros::Imu imu(2);

pros::Rotation hrotation(1);
pros::Rotation vrotation(-7);
lemlib::TrackingWheel horizontal_tracking_wheel(&hrotation, lemlib::Omniwheel::NEW_2, -1.377);
lemlib::TrackingWheel vertical_tracking_wheel(&vrotation, lemlib::Omniwheel::NEW_2, 0.114); // need to change

pros::adi::Pneumatics littlewill('E', false);
pros::adi::Pneumatics chickenstars('D', false);
pros::adi::Pneumatics midgoal('B', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 25 holes?
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4.29543 , // proportional gain (kP) 5.58 4.15
                                            0, // integral gain (kI) 
                                            5.9, // derivative gain (kD) 19.05
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.55, // proportional gain (kP) 
                                             0, // integral gain (kI) 
                                             8.95, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(
    // &vertical, // vertical tracking wheel
                        &vertical_tracking_wheel, // &vertical, // vertical tracking wheel
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
