#include "main.h"
#include "auton_paths.hpp"
//left
void left_auto(){
         chassis.setPose(0,0,0);
         chassis.moveToPoint(0, 36, 2000, {.maxSpeed = 60});
         pros::delay(10);
         chassis.turnToHeading(-90, 1000);
         littlewill.toggle();
         intakeone(12000);
         chassis.moveToPoint(-17, 36,1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 50});
         chassis.moveToPoint(-19, 36,1000, {.forwards = true});
         pros::delay(700);
         intakeall(0);
         pros::delay(100);
         chassis.moveToPoint(24, 36.5, 1000, {.forwards = false, .maxSpeed = 60});
         pros::delay(900);
         intakeall(12000);
}

//right
void right_auto() {
    intakeone(12000);
     chassis.moveToPoint(0, 19.511, 500, {.maxSpeed = 90});
     chassis.turnToHeading(15, 200);
     pros::delay(500);

     chassis.moveToPoint(5.273, 35.067, 750, {.maxSpeed = 50});
     pros::delay(500);
         littlewill.toggle();
     chassis.turnToPoint(29.375, 12.919, 500); // 31 12
     chassis.moveToPoint(29.375, 12.919, 1000, {.maxSpeed = 90});

     chassis.turnToHeading(180, 500);
     chassis.moveToPoint(29.8, 0, 500, {.maxSpeed = 60});
     pros::delay(1000);
     chassis.moveToPoint(30.8, -2, 500, {.maxSpeed = 60});
     pros::delay(1000);
    chassis.moveToPoint(30.8, 32, 1000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1200);
    chassis.moveToPoint(30.8, 34, 1000, {.forwards = false, .maxSpeed = 90});
    intakeall(12000);
    pros::delay(1000); // commit
    intakeall(-12000);
    pros::delay(300);
    intakeall(12000);
    pros::delay(2400);
    intakeall(0);
    chassis.moveToPoint(20.5, 25.128, 1000);// og 20.5
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(20.5, 44, 2000, {.forwards = false, .maxSpeed = 50});
    pros::delay(10); 
}

//sawp

void sawp(){
    chassis.setPose(0, 0, 90);
    chassis.moveToPoint(40.076, 0, 1000 , {.maxSpeed = 90});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(40.076, -10.81, 1000, {.maxSpeed = 90}); // move to matchload>
    pros::delay(1050);


    chassis.moveToPoint(40.076, 15.556, 1200, { .forwards = false ,.maxSpeed = 90});
    intakeall(12000);
    pros::delay(2100);
    intakeone(12000);
    chassis.moveToPoint(40.076, 0, 1500 , {.maxSpeed = 90}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();
    chassis.moveToPoint(15.292, 24.257, 500, {.maxSpeed = 90});
    pros::delay(1200);
    //chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-90, 1000);


    chassis.moveToPoint(-29.53, 24.257, 1000 , {.forwards = true, .maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();


    chassis.turnToHeading(225, 500);
    chassis.moveToPoint(-16.347, 35.594, 2000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1000);
    intakeone(0);
    chassis.turnToHeading(225, 200);
    pros::delay(100);
    intakemiddle(7000);
    pros::delay(500);
    intakeone(0);
    pros::delay(100);
  
}
//skills
void skills (){
chassis.setPose(0, 0, 90);
    chassis.moveToPoint(40.076, 0, 1000 , {.maxSpeed = 90});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(40.076, -10.81, 1000, {.maxSpeed = 90}); // move to matchload>
    pros::delay(1050);


    chassis.moveToPoint(40.076, 15.556, 1200, { .forwards = false ,.maxSpeed = 90});
    intakeall(12000);
    pros::delay(2100);
    intakeone(12000);
    chassis.moveToPoint(40.076, 0, 1500 , {.maxSpeed = 90}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();
    chassis.moveToPoint(15.292, 24.257, 500, {.maxSpeed = 90});
    pros::delay(1200);
    //chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-90, 1000);


    chassis.moveToPoint(-29.53, 24.257, 1000 , {.forwards = true, .maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();


    chassis.turnToHeading(225, 500);
    chassis.moveToPoint(-16.347, 35.594, 2000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1000);
    intakeone(0);
    chassis.turnToHeading(225, 200);
    pros::delay(100);
    intakemiddle(7000);
    pros::delay(500);
    intakeone(0);
    pros::delay(100);
}
//left elims
void left_elims(){

}
//right elims
void right_elims(){
    
}

