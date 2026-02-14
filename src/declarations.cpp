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
pros::MotorGroup leftMotors({15, -11, -16}, pros::MotorGearset::blue);   
pros::MotorGroup rightMotors({-18, 20, 13}, pros::MotorGearset::blue);  

// top right is 1 - acc 17 real 18
// bottom back right is 6 - acc 9 real 69 20
// bottom front right is  - acc 4 real 13

// top left is 15 - acc 10 real 15
// bottom back left is 8 - acc 11 real  16
// bottom front left is 14 - acc 3 real 11

pros::adi::Button autonselectbutton('C');


//intake mototro
pros::Motor intmotor1(-12); // first stage // 4 12
pros::Motor intmotor3(-19); // top // 19

// Inertial Sensor on port 19
pros::Imu imu(2);

pros::Rotation hrotation(10);
pros::Rotation vrotation(1);
lemlib::TrackingWheel horizontal_tracking_wheel(&hrotation, lemlib::Omniwheel::NEW_275, -1.377);//negative
lemlib::TrackingWheel vertical_tracking_wheel(&vrotation, lemlib::Omniwheel::NEW_2, 0.114); // need to change

pros::adi::Pneumatics littlewill('B', false);
pros::adi::Pneumatics chickenstars('A', false);
pros::adi::Pneumatics midgoal('C', false);

pros::Distance sensor1(12);
pros::Distance sensor2(20); 
pros::Distance sensor3(21); 
pros::Distance sensor4(19);

double sideSensorReading = 1;
double c = 1;
double  a = 1;
double  opposite = 1;
double distanceBetweenBackSensors = 172;
double  trackingCentreToWallSide = 1;
double  centreToWallSideAccount4Angle = 1;
double  distanceFromCentreBack = 1;
double  distanceFromCentreBackAccount4Angle = 1;
double  width = 279.4; // 13.5? // 11.5? // 12?
double  length = 292.1; //  15?
double  theta = 1;

double min_x = 1;
double min_y = 1;

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
                        nullptr, // &vertical, // vertical tracking wheel
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
