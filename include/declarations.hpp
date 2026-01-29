#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller master;

extern pros::MotorGroup leftMotors;  
extern pros::MotorGroup rightMotors;  

extern pros::adi::Button autonselectbutton;

extern pros::Motor intmotor1; // first stage // 20 
extern pros::Motor intmotor3; // top // 41

extern pros::Imu imu;

extern pros::Rotation hrotation;
extern pros::Rotation vrotation;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

extern pros::adi::Pneumatics littlewill;
extern pros::adi::Pneumatics chickenstars;
extern pros::adi::Pneumatics midgoal;

// extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance sensor1;
extern pros::Distance sensor2; 
extern pros::Distance sensor3; 
extern pros::Distance sensor4; 

extern double d;
extern double c;
extern double a;
extern double w;   
extern double b;
extern double d2;
extern double e2;
extern double y2;
extern double x2;
extern double width;
extern double theta;   
extern double length;