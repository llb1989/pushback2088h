#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "auton_paths.hpp"
//left
void left_auto(){

    chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90});
     chassis.turnToHeading(-15, 200);

    chassis.moveToPoint(-7.273, 41.067, 1000, {.maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(-34.5, 10,700, {.maxSpeed = 80});
    chassis.moveToPoint(-34.5, 10, 1500);
    chassis.turnToHeading(180, 1150, {.maxSpeed = 70});
    
    chassis.turnToHeading(180, 300, {.maxSpeed = 70});
    // chassis.moveToPoint(-34.5, -5, 200, {.maxSpeed = 90});
    // pros::delay(400);
    chassis.moveToPoint(-34.5, -8, 300, {.maxSpeed = 90});
    pros::delay(950);

    chassis.moveToPoint(-35, 35, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.moveToPoint(-35, 38, 300, {.forwards = false, .maxSpeed = 70});
    pros::delay(1300);
    intakeall(12000);
    pros::delay(900); // commit
    intakeall(12000);
    pros::delay(2000);
    intakeall(800);
    littlewill.toggle();
    chassis.moveToPoint(-49, 18, 1000);// og 20.5
    chassis.turnToHeading(180, 400);
    chassis.moveToPoint(-44, 45, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(180, 100);
    pros::delay(10); 
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
     chassis.turnToHeading(15, 200);
     pros::delay(500);

    intakeone(12000);
     chassis.moveToPoint(5.273, 40.067, 1000, {.maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(31.5, 18,600, {.maxSpeed = 90});
    chassis.moveToPoint(31.5, 18, 1000);

    chassis.turnToHeading(180, 1050);
    pros::delay(100); 
    intakeone(12000);
    chassis.moveToPoint(32.5, 2, 1000, {.maxSpeed = 100});
    pros::delay(200);
    chassis.moveToPoint(32.5, -0.5, 100, {.minSpeed = 80});
    pros::delay(300);

    chassis.turnToHeading(180, 300);
    chassis.moveToPose(34, 41, 180, 1100, {.forwards = false, .minSpeed = 60});
    pros::delay(1000);
    intakeall(12000);
    chassis.moveToPoint(34, 42, 100, {.minSpeed = 60});
    pros::delay(1500); // commit
    intakeall(-12000);
    pros::delay(300); // commit
    intakeall(12000);
    pros::delay(1400);
    intakeall(0);
    chassis.moveToPoint(21.5, 28, 1000, {.minSpeed = 60});// og 20.5
    chassis.moveToPoint(24.2, 48, 2000, {.forwards = false, .maxSpeed = 90, .minSpeed = 60});
    chassis.turnToHeading(150, 300);
    pros::delay(10); //
}

//sawp

void sawp(){

    chassis.setPose(0, 0, 90);
    chassis.moveToPoint(43, 0, 1100 , {.maxSpeed = 90});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 800); // turn to matchload>
    pros::delay(20);
    chassis.moveToPoint(43, -14.5, 800, {.maxSpeed = 90}); // move to matchload>
    chassis.moveToPoint(43, -15, 100, {.minSpeed =  70}); // move to matchload>
    pros::delay(650);

    // chassis.moveToPoint(41, 0, 600, {.maxSpeed = 90});

    chassis.moveToPoint(44, 22, 1200, {.forwards = false ,.maxSpeed = 100});
    pros::delay(900);

    intakeall(12000);
    pros::delay(200);
    intakeall(12000);
    pros::delay(600);
    intakeall(0);
    pros::delay(20);
    intakeone(12000);
    pros::delay(50);
    chassis.moveToPoint(36, 0, 700 , {.maxSpeed = 90}); // pull out?
    chassis.turnToHeading(-45, 400);
    
    littlewill.toggle();
    chassis.moveToPoint(18.8, 25, 1000, {.maxSpeed = 90});
    pros::delay(400);
    chassis.turnToHeading(-90, 500);

    chassis.moveToPoint(-30.5, 21, 1000 , {.forwards = true, .maxSpeed = 90});
    pros::delay(1200);
    littlewill.toggle();
    intakeone(0);

    chassis.turnToHeading(225, 400);
    chassis.moveToPoint(-13.5, 33.5, 1200, {.forwards = false, .maxSpeed = 100});
    pros::delay(500);
    chassis.turnToHeading(225, 300);
    intakemiddle(12000);
    pros::delay(250);
    intakeall(-12000);
    pros::delay(100);
    intakemiddle(12000);
    pros::delay(450);
    intakeone(12000);
    pros::delay(50);
    chassis.moveToPoint(-53, 4, 1600, {.minSpeed = 50});
    chassis.turnToHeading(180, 550);
    // chassis.moveToPoint(-54, -14, 700, {.minSpeed = 70});
    // pros::delay(800);
    chassis.moveToPoint(-53, 20, 700 , {.forwards = false, .maxSpeed = 120, .minSpeed = 50});
    pros::delay(750);
    intakeall(12000);

    // prac field BLUE
    // chassis.setPose(0, 0, 90);  
    // chassis.moveToPoint(42, 0, 1200 , {.maxSpeed = 90});
    // littlewill.toggle();
    // intakeone(12000);
    // chassis.turnToHeading(180, 700); // turn to matchload>
    // pros::delay(20);
    // chassis.moveToPoint(42, -14, 900, {.maxSpeed = 90}); // move to matchload>
    // chassis.moveToPoint(40, -14.5, 50, {.minSpeed =  70}); // move to matchload>
    // pros::delay(400);

    // // chassis.moveToPoint(41, 0, 600, {.maxSpeed = 90});

    // chassis.moveToPoint(42.5, 20, 1200, {.forwards = false ,.maxSpeed = 100});
    // pros::delay(900);

    // intakeall(12000);
    // pros::delay(1100);
    // intakeone(12000);
    // chassis.moveToPoint(36, 0, 500 , {.maxSpeed = 90}); // pull out?
    // chassis.turnToHeading(-45, 350);
    
    // littlewill.toggle();
    // chassis.moveToPoint(18.8, 25, 700, {.maxSpeed = 90});
    // pros::delay(400);
    // //chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
    // chassis.turnToHeading(-90, 500);

    // chassis.moveToPoint(-32.5, 22, 1000 , {.forwards = true, .maxSpeed = 90});
    // pros::delay(1200);
    // littlewill.toggle();

    // chassis.turnToHeading(225, 400);
    // chassis.moveToPoint(-18, 33, 1200, {.forwards = false, .maxSpeed = 100});
    // pros::delay(600);
    // intakeone(0);
    // chassis.turnToHeading(225, 300);
    // intakemiddle(12000);
    // pros::delay(800);
    // intakeone(1200);
    // pros::delay(100);
    // chassis.moveToPoint(-52, 4, 1600, {.minSpeed = 50});
    // chassis.turnToHeading(180, 550);
    // // chassis.moveToPoint(-54, -14, 700, {.minSpeed = 70});
    // // pros::delay(800);
    // chassis.moveToPoint(-52, 16, 700 , {.forwards = false, .maxSpeed = 120, .minSpeed = 50});
    // pros::delay(750);
    // intakeall(12000);
  
}
//skills
void skills (){

    intakeall(12000);
    forwards(12000, 12000);
    pros::delay(1000);
    forwards(0, 0);

// chassis.setPose(0, 0, 90);
//     chassis.moveToPoint(42, 0, 1100 , {.maxSpeed = 90});
//     littlewill.toggle();
//     intakeone(12000);
//     chassis.turnToHeading(180, 800); // turn to matchload>
//     pros::delay(20);
//     chassis.moveToPoint(42, -14.5, 00, {.maxSpeed = 90}); // move to matchload>
//     pros::delay(1000);
//     chassis.moveToPoint(42, -15, 100, {.minSpeed =  70}); // move to matchload>
//     pros::delay(1000);
//     chassis.moveToPoint(42, -15.75, 100, {.minSpeed =  70}); // move to matchload>
//     pros::delay(1000);

//     // chassis.moveToPoint(41, 0, 600, {.maxSpeed = 90});

//     chassis.moveToPoint(42, 20, 1200, {.forwards = false ,.maxSpeed = 70});
//     pros::delay(900);

//     intakeall(12000);
//     pros::delay(1000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000);   
//     pros::delay(3000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000); 
//     pros::delay(1000);
//      intakeone(8000);
//     chassis.moveToPoint(36, 0, 500 , {.maxSpeed = 70}); // pull out?
//     chassis.turnToHeading(-45, 350);
   
//     littlewill.toggle();
//     chassis.moveToPoint(18.8, 25, 700, {.maxSpeed = 70});
//     pros::delay(400);
//     chassis.turnToHeading(-90, 500);


//     chassis.moveToPoint(-32.5, 22, 1000 , {.forwards = true, .maxSpeed = 70});
//     pros::delay(1200);


//     // // chassis.turnToHeading(225, 400);
//     // chassis.moveToPoint(-16, 30, 1200, {.forwards = false, .maxSpeed = 100});
//     // pros::delay(600);
//     // intakeone(0);
//     // chassis.turnToHeading(225, 300);
//     // intakemiddle(12000);
//     // pros::delay(1000);
//     // intakeone(12000);
//     // pros::delay(100);

//     chassis.moveToPoint(-57, 4, 2200, {.maxSpeed = 70});
//     chassis.turnToHeading(180, 1200);
//     chassis.moveToPoint(-57, 4, 200, {.maxSpeed = 70});
//     chassis.turnToHeading(180, 200);
//     chassis.moveToPoint(-57, 16, 1500 , {.forwards = false, .maxSpeed = 70});
//     pros::delay(750);
//     intakeall(12000);
//     pros::delay(1000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000);   
//     pros::delay(3000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000);
//     littlewill.toggle();
    
//     intakeone(12000);
//     chassis.moveToPoint(-56.5, -16, 1000, {.maxSpeed = 70});
//     pros::delay(1000);
//     chassis.moveToPoint(-56.5, -16.5, 1000, {.maxSpeed = 70});
//     pros::delay(1000);
//     chassis.moveToPoint(-56.5, -17, 1000, {.maxSpeed = 70});
//     pros::delay(1000);
//     chassis.moveToPoint(-57, 20, 1000, {.forwards = false, .maxSpeed = 120, .minSpeed = 50});
//     pros::delay(750);
//     intakeall(12000);
//     pros::delay(1000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000);   
//     pros::delay(3000);
//     intakeall(-12000);
//     pros::delay(300);
//     intakeall(12000);

//     littlewill.toggle();
//     chassis.turnToPoint(-30, -13, 900);
//     chassis.moveToPoint(-30, -13, 1000);
//     chassis.turnToHeading(90, 1000);
//     chassis.cancelAllMotions();
//     forwards(12000, 12000);
//     pros::delay(800);
//     forwards(0, 0);


// chassis.setPose(0, 0, 0);
// intakeone(12000);
// chassis.moveToPoint(0, 40, 180, {.maxSpeed = 40});
// littlewill.toggle();
// chassis.turnToHeading(270, 500);
// chassis.moveToPoint(-20, 40, 1000, {.maxSpeed = 40});
// pros::delay(2000); //matchload
// chassis.moveToPoint(25, 40, 1000, {.forwards = false, .maxSpeed = 40});
// intakeall(12000);
// chassis.moveToPoint(0, 40, 1000, {.maxSpeed = 40});
// chassis.turnToHeading(180, 500);
// chassis.moveToPoint(0, 70, 1000, {.forwards = false, .maxSpeed = 40});

// pros::delay(1000);
// chassis.setPose(chassis.getPose().x, 60, chassis.getPose().theta); // wall reset
// pros::delay(1000);

// intakeone(12000);
// chassis.turnToHeading(90, 500);
// chassis.moveToPoint(95, 60, 1000, {.forwards = true, .maxSpeed = 40});

// chassis.turnToHeading(180, 500);
// pros::delay(1000);
// chassis.setPose(chassis.getPose().x, 60, chassis.getPose().theta); // wall reset
// pros::delay(1000);

// chassis.moveToPoint(95, 40, 1500, {.maxSpeed = 40});
// chassis.moveToPoint(120,40, 1000, {.maxSpeed = 40});
// pros::delay(2000); //matchload
// chassis.moveToPoint(70, 40, 1000, {.forwards = false, .maxSpeed = 40});
// pros::delay(1000);
// intakeall(12000);
// pros::delay(1000);

// intakeone(12000);
// chassis.turnToPoint(100, -55,500);
// chassis.moveToPoint(100, -55,3000, {.forwards = true, .maxSpeed = 40});
// chassis.turnToHeading(90, 500);
// chassis.moveToPoint(120, -55, 2000, {.forwards = true, .maxSpeed = 40});
// pros::delay(2000); //matchload
// chassis.moveToPoint(70, -55, 2000, {.forwards = false, .maxSpeed = 40});

// littlewill.toggle();
// chassis.swingToPoint(100, -75, lemlib::DriveSide::RIGHT, 2000); // freaky
// chassis.moveToPoint(100, -75, 2000, {.forwards = true, .maxSpeed = 40});

// pros::delay(1000);
// chassis.setPose(chassis.getPose().x, -75, chassis.getPose().theta); // wall reset
// pros::delay(1000);

// intakeone(12000);
// chassis.turnToHeading(270, 500);
// chassis.moveToPoint(0, -75, 3000, {.maxSpeed = 40});
// chassis.moveToPoint(0, -55, 1000, {.maxSpeed = 40});
// littlewill.toggle();
// chassis.moveToPoint(-20, -55, 1000, {.forwards = true, .maxSpeed = 40});
// pros::delay(2000); //matchload
// chassis.moveToPoint(25, -55, 1000, {.forwards = false, .maxSpeed = 40});


// chassis.moveToPoint(-15, -35, 1000, {.maxSpeed = 40});
// chassis.turnToHeading(0, 500);
// chassis.moveToPoint(-15, 5, 1000, {.maxSpeed = 90}); // park


// chassis.setPose(0, 0, 90);
//     chassis.moveToPoint(40.076, 0, 1000 , {.maxSpeed = 90});
//     littlewill.toggle();
//     intakeone(12000);
//     chassis.turnToHeading(180, 900); // turn to matchload>
//     chassis.moveToPoint(40.076, -10.81, 1000, {.maxSpeed = 90}); // move to matchload>
//     pros::delay(1050);


//     chassis.moveToPoint(40.076, 15.556, 1200, { .forwards = false ,.maxSpeed = 90});
//     intakeall(12000);
//     pros::delay(2100);
//     intakeone(12000);
//     chassis.moveToPoint(40.076, 0, 1500 , {.maxSpeed = 90}); // pull out?
//     chassis.turnToHeading(-45, 500);
//     littlewill.toggle();
//     chassis.moveToPoint(15.292, 24.257, 500, {.maxSpeed = 90});
//     pros::delay(1200);
//     //chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
//     chassis.turnToHeading(-90, 1000);


//     chassis.moveToPoint(-29.53, 24.257, 1000 , {.forwards = true, .maxSpeed = 90});
//     pros::delay(500);
//     littlewill.toggle();


//     chassis.turnToHeading(225, 500);
//     chassis.moveToPoint(-16.347, 35.594, 2000, {.forwards = false, .maxSpeed = 90});
//     pros::delay(1000);
//     intakeone(0);
//     chassis.turnToHeading(225, 200);
//     pros::delay(100);
//     intakemiddle(7000);
//     pros::delay(500);
//     intakeone(0);
//     pros::delay(100);
}
//left elims
void left_elims(){

}
//right elims
void right_elims(){

}

// void tuning_pid (){
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, 24, 10000);
// }
