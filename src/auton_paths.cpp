#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "auton_paths.hpp"
<<<<<<< HEAD


// void driveRelative(double distance, int timeout) {
//     lemlib::Pose pose = chassis.getPose();


//     double headingRad = pose.theta * M_PI / 180.0;


//     double targetX = pose.x + distance * cos(headingRad);
//     double targetY = pose.y + distance * sin(headingRad);


//     chassis.moveToPose(targetX, targetY, pose.theta, timeout);
// }



void eztemplate_test() {
    // chassis.turnToHeading(45, 1000);
    // driveRelative(24, 1000);
}

=======
float Maxspeed = 127;
float Minspeed = 0;
bool Forwards = true;
        void driveRelative(double distance, int timeout, float Maxspeed = 127, float Minspeed = 0, bool Forwards = true) {
    lemlib::Pose pose = chassis.getPose();

    double headingRad = pose.theta;
    double targetX = pose.x + distance * cos(headingRad);
    double targetY = pose.y + distance * sin(headingRad);

    chassis.moveToPose(targetX, targetY, pose.theta, timeout, {.forwards = Forwards, .maxSpeed = Maxspeed, .minSpeed = Minspeed});
}

void test(){
    chassis.turnToHeading(50, 500);
    driveRelative(50, 1000, 127, 127);
    // chassis.turnToHeading(90, 1000);
    // driveRelative(5, 1000);
}
>>>>>>> 39567459b4931649eb1259965cc11ff2d8348a54
//left
void left_auto(){
    pros::delay(200);
    chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90, .minSpeed = 40});
     chassis.turnToHeading(-15, 200);

    chassis.moveToPoint(-7.273, 41.067, 1000, {.maxSpeed = 90, .minSpeed = 40});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(-34.5, 10,700, {.maxSpeed = 90});
    chassis.moveToPoint(-33.5, 10, 1500);
    chassis.turnToHeading(180, 1150, {.maxSpeed = 90});
   
    chassis.turnToHeading(175, 300, {.maxSpeed = 90});
    // chassis.moveToPoint(-34.5, -5, 200, {.maxSpeed = 90});
    // pros::delay(400);
    intakeone(12000);
    chassis.moveToPoint(-34, -2, 400, {.maxSpeed = 90});
    pros::delay(200);
    chassis.moveToPoint(-34, -3, 400, {.maxSpeed = 90});
    pros::delay(300);
    chassis.moveToPoint(-34, -3.5, 400, {.maxSpeed = 90});

    chassis.moveToPoint(-34, 35, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 40});
    chassis.moveToPoint(-34, 38, 200, {.forwards = false, .maxSpeed = 100});
    pros::delay(100);
    intakeall(12000);
    pros::delay(900); // commit
    intakeall(12000);
    pros::delay(1500);
    intakeall(800);
    littlewill.toggle();
    chassis.moveToPoint(-48, 15, 1000);// og 20.5
    chassis.turnToHeading(180, 400);
    chassis.moveToPoint(-43, 45, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(70, 100);
    pros::delay(200); 
    rightMotors.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode(pros::MotorBrake::hold);
}

//right
void right_auto() { 
    // prac field BLUE
    // chassis.setPose(0, 0, 0);
    // intakeone(12000);
    //  chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90});
    //  chassis.turnToHeading(15, 200);
    //  pros::delay(500);

    // chassis.moveToPoint(5.273, 38.067, 1000, {.maxSpeed = 90});
    // pros::delay(500);
    // littlewill.toggle();
    // chassis.turnToPoint(32, 18,600, {.maxSpeed = 90});
    // chassis.moveToPoint(32, 18, 1000);

    // chassis.turnToHeading(180, 1050);
    // pros::delay(100); 
    // chassis.moveToPoint(32, -3, 1000, {.maxSpeed = 100});
    // chassis.turnToHeading(180, 700);
    // pros::delay(400);
    // chassis.moveToPoint(32, -4, 100, {.minSpeed = 70});
    // pros::delay(300);
    
    // chassis.moveToPoint(32, 35, 1000, {.forwards = false, .maxSpeed = 60});
    // chassis.turnToHeading(180, 300);
    // chassis.moveToPoint(32, 37, 800, {.forwards = false, .maxSpeed = 70});
    // pros::delay(1000);
    // intakeall(12000);
    // pros::delay(1500); // commit
    // intakeall(-12000);
    // pros::delay(300); // commit
    // intakeall(12000);
    // pros::delay(1500);
    // intakeall(0);
    // chassis.moveToPoint(19, 18.128, 1000);// og 20.5
    // chassis.turnToHeading(180, 300);
    // chassis.moveToPoint(20, 47, 2000, {.forwards = false, .maxSpeed = 50});
    // pros::delay(10); //

    // right field BLUE
    chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90});
     chassis.turnToHeading(15, 150);
     pros::delay(400);

    intakeone(12000);
     chassis.moveToPoint(5.273, 40.067, 1000, {.maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(31.8, 18,600, {.maxSpeed = 90});
    chassis.moveToPoint(31.8, 18, 1000);

    chassis.turnToHeading(180, 1050);
    pros::delay(50); 
    intakeall(-12000);
    pros::delay(100);
    intakeone(12000);
    chassis.moveToPoint(33, 2, 900, {.maxSpeed = 100});
    pros::delay(200);
    chassis.moveToPoint(33.5, -1, 200, {.minSpeed = 80});
    pros::delay(350);

    chassis.turnToHeading(180, 300);
    chassis.moveToPoint(33.8, 41, 1200, {.forwards = false, .minSpeed = 40});
    pros::delay(1000);
    intakeall(12000);
    // chassis.moveToPoint(35.5, 43, 100, {.minSpeed = 60});
    pros::delay(1500); // commit
    intakeall(-12000);
    pros::delay(300); // commit
    intakeall(12000);
    pros::delay(1400);
    intakeall(0);
    chassis.moveToPoint(22.5, 28, 1200, {.minSpeed = 65});// og 20.5
    chassis.moveToPoint(24.6, 48, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 65});
    chassis.turnToHeading(150, 1000, {.maxSpeed = 40});
    pros::delay(10); //
}

void leftmirrored(){
    chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90, .minSpeed = 40});
     chassis.turnToHeading(15, 200);

    chassis.moveToPoint(7.273, 41.067, 1000, {.maxSpeed = 90, .minSpeed = 40});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(34.5, 10,700, {.maxSpeed = 90});
    chassis.moveToPoint(33.5, 10, 1500);
    chassis.turnToHeading(180, 1150, {.maxSpeed = 90});
   

    chassis.turnToHeading(-175, 300, {.maxSpeed = 90});
    // chassis.moveToPoint(-34.5, -5, 200, {.maxSpeed = 90});
    // pros::delay(400);
    intakeone(12000);
    chassis.moveToPoint(34, -2, 400, {.maxSpeed = 90});
    pros::delay(200);
    chassis.moveToPoint(34, -3, 400, {.maxSpeed = 90});
    pros::delay(300);
    chassis.moveToPoint(34, -3.5, 400, {.maxSpeed = 90});

    chassis.moveToPoint(34, 35, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 40});
    chassis.moveToPoint(34, 38, 200, {.forwards = false, .maxSpeed = 100});
    pros::delay(100);
    intakeall(12000);
    pros::delay(900); // commit
    intakeall(12000);
    pros::delay(1500);
    intakeall(800);
    littlewill.toggle();
    chassis.moveToPoint(22, 15, 1000);// og 20.5
    chassis.turnToHeading(180, 400);
    chassis.moveToPoint(28, 45, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-70, 100);
    pros::delay(200); 
    rightMotors.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode(pros::MotorBrake::hold);

}

//sawp

void sawp(){
    pros::delay(20);
    chassis.setPose(0, 0, 0);
    intakeone(12000);
    littlewill.toggle();
    chassis.moveToPoint(0, 40, 1200, {.maxSpeed = 100});
    pros::delay(10);
    chassis.turnToHeading(90, 400);
    chassis.waitUntilDone();

    chassis.moveToPoint(21.5, 37.5, 1150, {.maxSpeed = 90});
    pros::delay(350);

    chassis.moveToPoint(-20, 38.5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intakeall(12000);
    pros::delay(1100);
    littlewill.toggle();
    intakeone(12000);

    chassis.moveToPoint(-24, 14, 800, {.maxSpeed = 80});
    chassis.waitUntilDone();

    chassis.moveToPoint(-20, -30.5, 1500, {.maxSpeed = 90, .minSpeed = 30});
    chassis.waitUntilDone();

    littlewill.toggle();
    //     chassis.turnToHeading(135, 350, {.minSpeed = 40});
    chassis.moveToPoint(-29, -20, 200, {.forwards = false, .maxSpeed = 90}); // suspiviouys

    chassis.moveToPoint(-37, -11.5, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(135, 350, {.minSpeed = 40});
    intakemiddle(12000);
    pros::delay(1600);
    intakeone(12000); 

    chassis.moveToPoint(5, -54, 1400, {.maxSpeed = 90, .minSpeed = 30});
    midgoal.set_value(false);
    intakeone(12000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 350, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveToPoint(30, -54, 1150, {.maxSpeed = 70, .minSpeed = 30});
    pros::delay(1000);
    // chassis.moveToPoint(27, -56, 400, {.minSpeed = 40});
    // pros::delay(600);
    chassis.moveToPoint(-16, -53, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intakeall(12000);

    // chassis.moveToPoint(-22, 34, 2000, {.forwards = true});
    // ToPoint(-22, 34, lemlib::DriveSide::LEFT, 2000, {.forwards = true});
    // ToPose(-22, 34, 190, 2000, {.forwards = true});
    // pros::delay(2000);
    // chassis.moveToPose(-24, -30, 180, 2000, {.forwards = true});


    // chassis.setPose(0, 0, 90);
    // chassis.moveToPoint(44, 0, 1100 , {.maxSpeed = 90});
    // littlewill.toggle();
    // intakeone(12000);
    // chassis.turnToHeading(180, 700); // turn to matchload>
    // pros::delay(20);
    // chassis.moveToPoint(43, -14.8, 800, {.maxSpeed = 100}); // move to matchload>
    // pros::delay(200);
    // chassis.moveToPoint(43, -15.8, 70, {.minSpeed =  70}); // move to matchload>
    // pros::delay(400);

    // // chassis.moveToPoint(41, 0, 600, {.maxSpeed = 90})vbj;

    // chassis.moveToPoint(44, 22, 1200, {.forwards = false ,.maxSpeed = 100});
    // pros::delay(800);

    // intakeall(12000);
    // pros::delay(450);
    // intakeall(-12000);
    // pros::delay(0);
    // intakeall(12000);
    // pros::delay(500);
    // intakeone(12000);
    // chassis.moveToPoint(36, 6, 700 , {.maxSpeed = 90, .minSpeed = 30}); // pull out?
    // chassis.turnToHeading(-45, 400);
    
    // littlewill.toggle();
    // chassis.moveToPoint(18.8, 25, 1000, {.maxSpeed = 100});
    // pros::delay(400);
    // chassis.turnToHeading(-90, 500);

    // chassis.moveToPoint(-31, 21, 850 , {.forwards = true, .maxSpeed = 100});
    // pros::delay(900);
    // littlewill.toggle();
    // intakeone(12000);

    // chassis.turnToHeading(225, 550);
    // chassis.moveToPoint(-12, 33, 1100, {.forwards = false, .maxSpeed = 100});
    // pros::delay(300);
    // chassis.turnToHeading(225, 200);
    // intakemiddle(12000);
    // pros::delay(200);
    // intakemiddle(-12000);
    // pros::delay(400);
    // intakemiddle(12000);
    // pros::delay(600);
    // intakeone(12000);
    // pros::delay(50);
    // chassis.moveToPoint(-51, 4, 1800, {.minSpeed = 30});
    // chassis.turnToHeading(180, 800);
    // chassis.moveToPoint(-50.5, 20, 700 , {.forwards = false, .maxSpeed = 120, .minSpeed = 30});
    // pros::delay(450);
    // intakeall(12000);
  
}

void left_and_mid_rush() {
    // // pros::delay(100);
    // pros::delay(200);
    // chassis.setPose(0, 0, 0);
    // intakeone(12000);
    // littlewill.toggle();
    // pros::delay(100);
    // chassis.moveToPoint(0, 39.5, 1250, {.maxSpeed = 80});
    // chassis.turnToHeading(-90, 300);
    // littlewill.toggle();
    // chassis.moveToPoint(0, 39, 1250, {.maxSpeed = 80});
    // chassis.turnToHeading(-90, 1000);

    // chassis.moveToPoint(-18, 37, 1000, {.maxSpeed = 90});
    // pros::delay(250);
    // chassis.moveToPoint(18, 39.5, 1000, {.forwards = false, .maxSpeed = 90});
    // chassis.waitUntilDone();
    // intakeall(12000);
    // pros::delay(1200);
    // intakeone(12000);
    // littlewill.toggle();
    // // chassis.moveToPoint(6, 40.2, 500, {.forwards = true, .maxSpeed = 90});

    // // chassis.turnToPoint(21, 14, 500);

    // chassis.moveToPoint(23, 10, 600, {.maxSpeed = 80});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(315, 300);
    // chassis.moveToPoint(36, 11, 1000, {.forwards = false, .maxSpeed = 40}); //help
    // chassis.turnToHeading(315, 200);
    // chassis.waitUntilDone();
    // intakemiddle(12000);
    // pros::delay(1000);

    // intakeone(12000);
    // littlewill.toggle();
    // chassis.moveToPoint(6,31.5, 1200, {.forwards = true});
    // chassis.turnToHeading(90, 500, {.maxSpeed = 90});
    // chassis.moveToPoint(35, 31.5, 2000, {.forwards = true, .maxSpeed = 90});
    // chassis.turnToHeading(150, 500, {.maxSpeed = 100});

    chassis.setPose(0, 0, 0);
    intakeone(12000);
        littlewill.toggle();
    chassis.moveToPoint(0, 39, 1250, {.maxSpeed = 100});
    chassis.turnToHeading(-90, 300);

    chassis.moveToPoint(-20, 37, 1000, {.maxSpeed = 90, .minSpeed = 30}); // was 37.5 overshoot
    pros::delay(550);
    // chassis.moveToPoint(16, 40, 1000, {.maxSpeed = 90});

    // pros::delay(200);
    chassis.moveToPoint(20, 36, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 40}); // 37 overshooted in match
    chassis.waitUntilDone();
    intakeall(12000);
    pros::delay(1200);
    intakeone(12000);
    littlewill.toggle();

    chassis.moveToPoint(25, 14, 1000, {.maxSpeed = 100, .minSpeed = 40});
    chassis.waitUntilDone();
    littlewill.toggle();


    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(31, 14, 810, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-45, 550);
    chassis.waitUntilDone();
    intakemiddle(12000);
    pros::delay(1500);
    intakeall(0);
    midgoal.set_value(true);
    littlewill.toggle();
    // chassis.moveToPoint(15, 25, 750, {.forwards = true});
    //     chassis.turnToHeading(-270, 300, {.maxSpeed = 60});
//     chassis.moveToPoint(12, 33.5, 750, {.forwards = true, .maxSpeed = 100, .minSpeed = 40});

//     chassis.moveToPoint(37, 45, 1000);
//     chassis.waitUntilDone();
//     chassis.setPose(0,0,0);
// chassis.moveToPose(0, 10, 0, 750);
//     chassis.turnToHeading(90, 500, {.maxSpeed = 100, .minSpeed = 40});
//     midgoal.set_value(false);

}

void right_goal_rush_skills() {
    //pros::delay(200);
        littlewill.toggle();
        intakeone(12000);
    chassis.setPose(0, 0, 0);
    


    chassis.moveToPoint(0, 37.7, 3000, {.maxSpeed = 60});
    chassis.turnToHeading(90, 700, {.maxSpeed = 60});

    chassis.moveToPoint(20.5 , 37.7, 1000, {.maxSpeed = 90});
    pros::delay(900);
    // chassis.moveToPoint(16, 40, 1000, {.maxSpeed = 90});
    // pros::delay(200);
    chassis.moveToPoint(-18, 38.5, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone(); 
    intakeall(1200);
    pros::delay(700);  //1300
    intakeall(-12000);
    pros::delay(150);
    intakeall(12000);
    pros::delay(3000);  //1300

    intakeone(12000);
    littlewill.toggle();

    chassis.moveToPoint(-24, 12, 1000);
    chassis.waitUntilDone();
    littlewill.toggle();

    chassis.moveToPoint(-17, 31.3, 500, {.forwards = false});
        chassis.turnToHeading(90, 300, {.maxSpeed = 60});
    chassis.moveToPoint(-34, 36.5, 1000, {.forwards = false, .maxSpeed = 80});
intakeall(12000);
    chassis.moveToPoint(-17, 31.3, 500, {.forwards = false});
    chassis.turnToPoint(20, 7, 2000, {.maxSpeed = 60});
        chassis.moveToPoint(20, 7, 2000, {.maxSpeed = 60});
        chassis.turnToHeading(160, 500);
        chassis.moveToPoint(20, -30, 2000, {.maxSpeed = 70});
        chassis.moveToPoint(17, -7, 2000, {.forwards = false, .maxSpeed = 70});
}

void right_goal_rush() {
    //pros::delay(200);
        littlewill.toggle();
        intakeone(12000);
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 41.5, 1700, {.maxSpeed = 70});
    chassis.turnToHeading(90, 700);

    chassis.moveToPoint(20.5 , 42, 1000, {.maxSpeed = 90});
    pros::delay(900);
    // chassis.moveToPoint(16, 40, 1000, {.maxSpeed = 90});
    // pros::delay(200);
    chassis.turnToHeading(90, 200);
    chassis.moveToPoint(-18, 42, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30});
    chassis.waitUntilDone();
    intakeall(1200);
    pros::delay(500);  //1300
    intakeall(0);
    pros::delay(100);
    intakeall(12000);
    pros::delay(4000);  //1300

    intakeone(12000);
    littlewill.toggle();

    chassis.moveToPoint(-21, 12, 1000);
    chassis.waitUntilDone();
    littlewill.toggle();

    chassis.moveToPoint(-9, 35, 700, {.forwards = false});
        chassis.turnToHeading(90, 300, {.maxSpeed = 60});
    chassis.moveToPoint(-23, 39, 780, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(40, 500);
    intakeall(0);
    leftMotors.set_brake_mode(pros::MotorBrake::hold);
    rightMotors.set_brake_mode(pros::MotorBrake::hold);
}

void right_mid(){
    chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90}); // forward
     chassis.turnToHeading(15, 200);
     pros::delay(500);

    intakeone(12000);
     chassis.moveToPoint(5.273, 40.067, 1000, {.maxSpeed = 90}); // block cluster thing
    pros::delay(500);
    chassis.turnToHeading(315, 500); // to goal
    chassis.moveToPoint(-5.96, 58.358, 1000, {.maxSpeed = 90});
    intakeall(-12000);
    pros::delay(500);
    intakeone(12000);

    chassis.moveToPoint(33, 18, 1500, {.forwards = false, .maxSpeed = 90}); // back out
    chassis.turnToHeading(180, 700);

    chassis.moveToPoint(33, 2, 1000, {.maxSpeed = 90});
    pros::delay(200);
    chassis.turnToHeading(180, 100);
    chassis.moveToPoint(33, 0, 1000, {.maxSpeed = 90});
    pros::delay(200);
    chassis.moveToPoint(33, 40, 1000, {.forwards = false, .maxSpeed = 90});
    pros::delay(100);
    intakeall(12000);
    pros::delay(500);

    // wing play

}


//skills
void skills (){

   chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90});
     chassis.turnToHeading(15, 150);
     pros::delay(400);

    intakeone(12000);
     chassis.moveToPoint(5.273, 40.067, 1000, {.maxSpeed = 90});
    pros::delay(2500);
    chassis.moveToPoint(6, 40.067, 100, {.maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(31.8, 18,600, {.maxSpeed = 90});
    chassis.moveToPoint(31.8, 18, 1000);

    chassis.turnToHeading(180, 1050);
    pros::delay(50); 
    intakeall(-12000);
    pros::delay(100);
    intakeone(12000);
    chassis.moveToPoint(33, 2, 900, {.maxSpeed = 100});
    pros::delay(200);
    chassis.moveToPoint(33.5, -1, 200, {.minSpeed = 80});
    pros::delay(350);

    chassis.turnToHeading(180, 300);
    chassis.moveToPoint(33.8, 41, 1200, {.forwards = false, .minSpeed = 40});
    pros::delay(1000);
    intakeall(12000);
    // chassis.moveToPoint(35.5, 43, 100, {.minSpeed = 60});
    pros::delay(1500); // commit
    intakeall(-12000);
    pros::delay(300); // commit
    intakeall(12000);
    pros::delay(1400);
    intakeall(0);
    chassis.moveToPoint(22.5, 28, 1200, {.minSpeed = 65});// og 20.5
    chassis.moveToPoint(24.6, 48, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 65});
    chassis.turnToHeading(150, 1000, {.maxSpeed = 40});
    pros::delay(10); //
 
        chassis.moveToPoint(-10, 30, 1200, {.forwards = true}); // out of goal

    littlewill.toggle(); // little will ups
    chassis.moveToPoint(-25, 11, 1000, {.maxSpeed = 80}); // back to 4 ball position
    chassis.waitUntilDone();

chassis.moveToPoint(-16, -33, 1500, {.maxSpeed = 90, .minSpeed = 30});
    chassis.waitUntilDone();

    littlewill.toggle();
    //     chassis.turnToHeading(135, 350, {.minSpeed = 40});
    // chassis.moveToPoint(-20, -21, 200, {.forwards = false, .maxSpeed = 90}); // suspiviouys

    chassis.moveToPoint(-33.5, -17.5, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(135, 350, {.minSpeed = 40});
    intakemiddle(12000);
    pros::delay(2000);
    intakeone(12000);

    // chassis.moveToPoint(-31.5, -22, 100, {.forwards = false, .maxSpeed = 90});
    // chassis.turnToHeading(135, 350, {.minSpeed = 40});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-30, -12, 1250, {.forwards = false, .maxSpeed = 90});
    // chassis.turnToHeading(135, 350, {.minSpeed = 40});
    // chassis.waitUntilDone();
    // intakemiddle(12000);
    // pros::delay(3000);
    // intakeone(12000);
    midgoal.set_value(true);

    chassis.moveToPoint(-5, -58, 2500, {.maxSpeed = 90});
    midgoal.set_value(false);
    intakeone(12000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, -57, 2000, {.forwards = false, .maxSpeed = 80});
        chassis.waitUntilDone();
    chassis.turnToHeading(90, 500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    intakeall(12000);
    pros::delay(2000);
    intakeone(12000);
    chassis.moveToPoint(19, -56, 2000, {.maxSpeed = 60});
    pros::delay(1500);
    chassis.moveToPoint(20, -56, 2000, {.maxSpeed = 60});
    pros::delay(1000);
    chassis.moveToPoint(20.5, -56, 2000, {.maxSpeed = 60});
    pros::delay(500);
    // chassis.moveToPoint(27, -56, 400, {.minSpeed = 40});
    // pros::delay(600);
    chassis.moveToPoint(-24, -54, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intakeall(12000);
    pros::delay(3000);   
    intakeone(12000);
    littlewill.toggle();
    chassis.moveToPoint(-10, -67, 1200, {.maxSpeed = 50}); //wing
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(-36, -64, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-10, -67, 1000, {.maxSpeed = 70});
 
intakeall(12000);
chassis.turnToPoint(10, -30, 1500);
chassis.moveToPoint(10, -30, 1500, {.maxSpeed = 60});
chassis.turnToHeading(20, 300);
forwards(12000, 12000);
pros::delay(1000);
forwards(0, 0);
// chassis.moveToPoint(20, -10, 1000);
// chassis.moveToPoint(10, -30, 3000, {.maxSpeed = 80});
// chassis.moveToPoint(10, 30, 2500);
// pros::delay(5000);
// intakeall(0);
// chassis.moveToPoint(10, -5, 1000, {.forwards = false});
// euwiwi
  
}
//left elims
void left_elims(){

}
//right elims
void right_elims(){

}

void skills_freaky() {

pros::delay(20);
    chassis.setPose(0, 0, 0);
    intakeone(12000);
    littlewill.toggle();
    chassis.moveToPoint(0, 40, 1500, {.maxSpeed = 70});
    pros::delay(10);
    chassis.turnToHeading(90, 400);
    chassis.waitUntilDone();

    chassis.moveToPoint(20.5, 37.5, 1150, {.maxSpeed = 90});
    pros::delay(1000);
    chassis.moveToPoint(21.5, 37.5, 250, {.maxSpeed = 90});
    pros::delay(500);
    chassis.moveToPoint(22, 37.5, 150, {.maxSpeed = 100});

    chassis.moveToPoint(-20, 41.5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intakeall(12000);
    pros::delay(3000);
    chassis.moveToPoint(-21, 40.5, 200, {.forwards = false, .maxSpeed = 80});
    pros::delay(1000);
    littlewill.toggle();
    intakeone(12000);

    chassis.moveToPoint(-20, 14, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();

    chassis.moveToPoint(-19, -30.5, 1500, {.maxSpeed = 90, .minSpeed = 30});
    chassis.waitUntilDone();

    chassis.turnToHeading(45, 500);
    pros::delay(100);
    chassis.moveToPoint(0, -46, 1400, {.maxSpeed = 90});
    intakeone(12000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    littlewill.toggle();
    chassis.moveToPoint(-16, -46, 1000, {.forwards = false, .maxSpeed = 80});
    intakeall(12000);
    pros::delay(3000);

    intakeone(12000);
    chassis.moveToPoint(30, -46, 1150, {.maxSpeed = 70, .minSpeed = 30});
    pros::delay(1500);
    chassis.moveToPoint(31, -46, 250, {.maxSpeed = 70, .minSpeed = 30});
    pros::delay(1500);
    // chassis.moveToPoint(27, -56, 400, {.minSpeed = 40});
    // pros::delay(600);
    chassis.moveToPoint(-16, -46, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intakeall(12000);


}