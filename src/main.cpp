#include "main.h"
#include "declarations.hpp"
#include "auton_functions.hpp"
#include "auton_paths.hpp"
#include "op_functions.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

std::string job = "thanks"; //lyla is the best most awesomesyt coder ever and nolan is a jewtwinkigga

    const int numAutos = 9;
int states[numAutos] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
int currAuto = 0;

void nextState() {
    currAuto += 1;
    if (currAuto == numAutos) {
        currAuto = 1;
    }
}

bool locktoggle = false;
// bool slowtoggle = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        
        while (true) {
        double drivetrainTemp = (rightMotors.get_temperature() + leftMotors.get_temperature()) / 2;

        pros::lcd::print(0, "cur X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "cur Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "lem imu theta: %f", chassis.getPose().theta); // heading

        // pros::lcd::print(6, "Rotation Sensor: %i", rotation.get_position());
        pros::lcd::print(3, "Temp: %0.1f", drivetrainTemp);
        // pros::lcd::print(3, "c: %d mm\n", sensor2.get());
        // pros::lcd::print(4, "a: %d mm\n", sensor3.get());
        // // pros::lcd::print(1, "dL: %d mm\n", sensor4.get());
        pros::lcd::print(5, "distance theta: %0.1f", theta); // heading
        pros::lcd::print(6, "distanceFromCentreBackAccount4Angle: %0.1f", distanceFromCentreBackAccount4Angle); 
        pros::lcd::print(7, "centreToWallSideAccount4Angle: %0.1f", centreToWallSideAccount4Angle); 
        
        // pros::lcd::print(3, "sensor 1 left: %d mm\n", sensor1.get());
        // pros::lcd::print(4, "sensor 4 right: %d mm\n", sensor4.get());
        // pros::lcd::print(5, "sensor 2: %d mm\n", sensor2.get());
        // pros::lcd::print(6, "sensor 3: %d mm\n", sensor3.get());

            if (currAuto == 1) {
            job = "right auto";
            } else if (currAuto == 2) {
            job = "left auto";
            } else if (currAuto == 3) {
            job = "alliance do sawp";
            } else if (currAuto == 4) {
            job = "100% working sawp";
            } else if (currAuto == 5){
            job = "better sawp";
            } else if (currAuto == 7){
            job = "skills";
            }else {
                job = "no auto selected";
            }

            // pros::lcd::print(3, "Auto: %d", currAuto);
            // pros::lcd::print(4, "Auto name: %s", job);
            // master.print(1, 2, "Auto: %d", currAuto);
            // master.print(1, 2, "Y: %f", chassis.getPose().y);
            //master.print(1, 2, "Auto?: %s", job);
            // pros::lcd::print(4, "Auto name: %s", job);

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
                master.print(1, 1, "Y: %f", chassis.getPose().y);

            // delay to save resources
            pros::delay(50);
            
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() { // auto select
        master.print(1, 1, "Y: %f", chassis.getPose().y);

    while (true) {
        if (autonselectbutton.get_new_press()) {
    nextState();
        }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    nextState();
    }
    // master.print(1, 2, "Auto: %f", currAuto);
}
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
 
/**
 * Runs during auto-mode
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    pros::delay(50); // calibrates it says dean
    master.print(1, 1, "Y: %f", chassis.getPose().y);

    int autonumber = currAuto;
    switch (7){
        case 1: // forwards // bleh

        test();
        break;
                                                                                                                                                                       
        case 2: // left
        left_auto();
        break;
    
        case 3: // 7+ wing right side
        right_auto();
        break;

        case 4: // true sawp
        sawp();
        break;

        case 5: // skillshhg
        skills();
        break;
 
        case 7:
        right_goal_rush();
        break;

        case 8:

        left_and_mid_rush();
        break;

        case 9:
        right_goal_rush_skills();
        break;

        case 10:
        chassis.setPose(0, 0, 0);
        pros::delay(100);
        chassis.moveToPoint(0, -24, 2000, {.forwards = false, .maxSpeed = 70});
        pros::delay(5000);
        lemreset(true, 1, false, true, true, 0, 15, -48, 0, true);
        break;

        case 11:
        chassis.setPose(0, 0, 0);
        pros::delay(1000);
        chassis.turnToHeading(90, 1000);
        chassis.turnToHeading(00, 1000);
        break;

        case 12:
        chassis.setPose(0, 0, 0);
        pros::delay(1000);
        chassis.moveToPoint(0, 48, 10000);
        break;
        
        case 13:
        // pros::delay(200);
        // chassis.setPose(0, 0, 0);
        // pros::delay(100);
        // intakeone(12000);
        // chassis.moveToPoint(0, 40, 1200, {.forwards = true, .maxSpeed = 80}); // first loader right side
        // littlewill.toggle();
        // chassis.waitUntilDone();
        // chassis.turnToHeading(90, 800);
        // chassis.moveToPoint(16, 40, 1000); //matchload
        // pros::delay(800);
        // intakeone(8000);
        // chassis.moveToPoint(17, 40, 1000); 
        // pros::delay(1500);
        // chassis.moveToPoint(0, 40, 1000, {.forwards = false, .maxSpeed = 80}); // reset epstein
        // chassis.turnToHeading(180, 1000);
        // chassis.moveToPoint(0, 68, 1500, {.forwards = false, .maxSpeed = 90}); // reset epstein
        // // intakeall(0);
        // chassis.waitUntilDone();
        // pros::delay(200);
        // lemreset(true, 3, true, false, true, 0, 16, -76, 58, true); // reset epstein
        // pros::delay(2000);
        // chassis.turnToHeading(270, 1000);

        // chassis.moveToPoint(-90, 59, 3500, {.forwards = true, .maxSpeed = 80}); // next quadrant
        // littlewill.toggle();
        // chassis.waitUntilDone();
        // chassis.waitUntilDone();
        // chassis.turnToHeading(180, 600);
        // chassis.moveToPoint(-90, 68, 600, {.forwards = false, .maxSpeed = 90}); // line up to goal
        // chassis.waitUntilDone();
        // littlewill.toggle();
        // pros::delay(200);
  
        // lemreset(true, 3, false, true, true, -120, 16, -76, 58, true); // reset the right
        // pros::delay(500);

        // chassis.moveToPoint(90, 41, 1000, {.maxSpeed = 80}); // make 40
        // chassis.turnToHeading(270, 800);
        // chassis.moveToPoint(-80, 41, 800, {.forwards = false, .maxSpeed = 110, .minSpeed = 50}); // score long goal
        // chassis.waitUntilDone();
        // pros::delay(300);
        // intakeall(12000);
        // pros::delay(1000);
        // intakeall(12000);
        // pros::delay(3000);
        // intakeone(12000);
        // chassis.turnToHeading(270, 400);
        // chassis.moveToPoint(-115, 41, 800, {.forwards = true, .maxSpeed = 80}); // matchload 
        // pros::delay(500);
        // intakeone(8000);
        // chassis.moveToPoint(-116.5, 41, 800, {.forwards = true, .maxSpeed = 90}); // matchload
        // pros::delay(2000);
        // chassis.moveToPoint(-80, 40, 1000, {.forwards = false, .maxSpeed = 110, .minSpeed = 50}); //score epstein
        // chassis.waitUntilDone();
        //  pros::delay(300);
        // intakeall(12000);
        // pros::delay(1000);
        // intakeall(-12000);
        // pros::delay(300);
        // intakeall(12000);
        // pros::delay(3000);
        // intakeall(-12000);
        // pros::delay(100);
        // intakeall(12000);
        // pros::delay(2000);
        // intakeone(12000);
        // chassis.moveToPoint(-100, 40, 1000, {.forwards = true, .maxSpeed = 80}); // set up to go other half
        // // littlewill.toggle();
        // chassis.turnToHeading(0, 600);
        // chassis.waitUntilDone();

        // intakeall(-12000); // why?????????
        // chassis.moveToPoint(-107, -80, 3500, {.forwards = false, .maxSpeed = 80}); // other half i think
        // chassis.turnToHeading(0, 600);
        // chassis.waitUntilDone();
        // littlewill.toggle(); // erase later ##########
        lemreset(true, 1, true, false, true, -120, 16, -73, 58, true); // reset the left
        pros::delay(200);
        intakeone(12000);
        chassis.moveToPoint(-100, -56, 800, {.forwards = true, .maxSpeed = 90});
        chassis.turnToHeading(270, 700);

        chassis.moveToPoint(-116, -54, 1000, {.forwards = true, .maxSpeed = 70}); // matchloader
        pros::delay(1400); 
        
        chassis.moveToPoint(-100, -55, 1000, {.forwards = false, .maxSpeed = 80});
        chassis.turnToHeading(0, 600);
        chassis.waitUntilDone(); 
        chassis.moveToPoint(-100, -90, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 30}); // set up go to other quadrant
        pros::delay(200);
        lemreset(true, 1, true, false, true, -120, 16, -73, 58, true);
        pros::delay(1000);

        chassis.turnToHeading(90, 600);
        littlewill.toggle();
        chassis.moveToPoint(-10, -78, 3500, {.forwards = true, .maxSpeed = 90}); // keep going 
        chassis.turnToHeading(0, 600); // og angle
        chassis.moveToPoint(-10, -84, 1000, {.forwards = false, .maxSpeed = 90});
        pros::delay(200);
        lemreset(true, 1, false, true, true, -120, 16, -73, 58, true); // reset 4th quad?
        pros::delay(500);
        chassis.turnToHeading(0, 200); // og angle
        chassis.moveToPoint(-10, -56, 1000, {.forwards = true, .maxSpeed = 90}); // move off wall reset
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 1000);
        littlewill.toggle();
        chassis.waitUntilDone();
        
        chassis.moveToPoint(-25, -56, 1000, {.forwards = false, .maxSpeed = 90}); // score left bottom
        chassis.waitUntilDone();
        intakeall(12000);
        pros::delay(2000);
        intakeone(12000);
        chassis.moveToPoint(16, -56, 1000, {.forwards = true, .maxSpeed = 50}); // matchload again i think
        pros::delay(500);
        chassis.moveToPoint(17.7, -56, 1000, {.forwards = true, .maxSpeed = 50}); // matchlaod again i think
        pros::delay(1000);

        chassis.moveToPoint(-25, -56, 1000, {.forwards = false, .maxSpeed = 90}); // score again left bottom
        chassis.waitUntilDone();
        intakeall(12000);
        pros::delay(3000);

        littlewill.toggle(); // heh
        chassis.moveToPoint(0, -60, 800, {.forwards = true});
        pros::delay(100);

        chassis.moveToPose(20, -16, 0, 2000, {.forwards = true}); // set up for park, moce towards
        chassis.waitUntilDone();
        pros::delay(200);
        lemreset(true, 1, false, true, false, -120, 16, -73, -58, true);
        pros::delay(500);
        chickenstars.toggle();
        chassis.moveToPoint(20, -18, 2000, {.forwards = true});
        littlewill.toggle();
        chassis.waitUntilDone();
        
        chassis.moveToPoint(20, 0, 2000, {.forwards = true}); // was -4


        // ## double clear ##
        // chassis.turnToHeading(-135, 800);
        // chassis.moveToPoint(-116, 30, 800);
        // chassis.turnToHeading(180, 600);
        // chassis.waitUntilDone();
        // lemreset(true, 3, false, true, true, -120, 16, -76, 58, true);
        // chassis.moveToPoint(-130, -20, 1000, {.forwards = true, .maxSpeed = 120, .minSpeed = 60});
        // chassis.turnToHeading(190, 800);
        // chassis.waitUntilDone();
        // forwards(12000, 12000);
        // pros::delay(3000);
        // forwards(0,0);
        // pros::delay(500);
        // lemreset(true, 3, false, true, true, -120, 16, -76, 58, true);
        // pros::delay(100);
        // chassis.turnToPoint(-100, -50, 600);
        // chassis.moveToPoint(-100, -50, 1000, {.forwards = true, .maxSpeed = 80});



        // max y is 58
        // min y is -76
        // min x is bleh -120
        // max x is 16 

        // one tile is 62mm 
        // 58.82
        // 57.97
        // acc = 2 tiles plus 23 inch and 1 quarter inch

        break;
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {

    while (true) {
        master.print(1, 1, "Y: %f", chassis.getPose().y);

        // get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        leftMotors.move_voltage((leftY + rightX) * 12000 / 127);
        rightMotors.move_voltage((leftY - rightX) * 12000 / 127);

        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && (pros::E_CONTROLLER_DIGITAL_UP)){ // run auto
        // autonomous();
        // } 

        if (autonselectbutton.get_new_press()) {
            nextState();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            nextState();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // turn lock on and off
            locktoggle = !locktoggle; 
        }

        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { // turn lock on and off
        //     slowtoggle = !slowtoggle; 
        // }

        if (locktoggle) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                intakeone(12000);
            } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intakemiddle(12000);
            } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intakeall(-12000);
            } else {
                intakeall(0);
            }  
        } else {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                intakeall(12000);
            } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intakemiddle(12000); //. intake middle meow
            } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intakeall(-12000);
            } else {
                intakeall(0);
            }
        }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        chickenstars.toggle();
    }
    
    if(master.get_digital_new_press(DIGITAL_Y)) {
        littlewill.toggle();
    }

    if(master.get_digital_new_press(DIGITAL_UP)) {
    lemreset(true, 1, true, false, true, -120, 16, -76, 58, true);
    }

    if(master.get_digital_new_press(DIGITAL_X)) {
    backdsr();  
    chassis.setPose(chassis.getPose().x, distanceFromCentreBackAccount4Angle, chassis.getPose().theta);

    }

        // delay to save resources
    pros::delay(10);
    }
}