#include "main.h"
#include "declarations.hpp"
#include "auton_functions.hpp"
#include "auton_paths.hpp"
#include "op_functions.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

std::string job = "thanks";

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
        pros::lcd::print(6, "x2: %0.1f", x2); 
        pros::lcd::print(7, "e2: %0.1f", e2); 
        
        pros::lcd::print(8, "sensor 1: %d mm\n", sensor1.get());
        pros::lcd::print(9, "sensor 4: %d mm\n", sensor4.get());

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
    master.print(1, 1, "Y: %f", chassis.getPose().y);

    int autonumber = currAuto;
    switch (8){

        case 1: // forwards
        forwards(8000, 8000);
        pros::delay(100);
        forwards(0, 0);
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

        case 5: // skills
        skills();
        break;

        case 7:
        right_goal_rush();
        break;

        case 8:
        left_and_mid_rush();
        break;

        case 6: //right 
        chassis.setPose(0, 0, 0);
    intakeone(12000);
     chassis.moveToPoint(0, 22, 500, {.maxSpeed = 90});
     chassis.turnToHeading(15, 200);
     pros::delay(500);

    chassis.moveToPoint(5.273, 38.067, 1000, {.maxSpeed = 40});
    pros::delay(500);
    littlewill.toggle();
    chassis.turnToPoint(30, 12,600, {.maxSpeed = 40});
    chassis.moveToPoint(30, 12, 1000);

    chassis.turnToHeading(180, 1050);
    chassis.moveToPoint(34, 36, 1000, {.forwards = false, .maxSpeed = 90});
    pros::delay(500);
    intakeall(12000);
     pros::delay(500);
     intakeone(12000);

    chassis.moveToPoint(34, -4, 700, {.maxSpeed = 90});
    chassis.turnToHeading(180, 500);
    pros::delay(400);
    chassis.moveToPoint(34, -5, 500, {.maxSpeed = 90});
    pros::delay(300);
    chassis.moveToPoint(37, 35, 1000, {.forwards = false, .maxSpeed = 50});
    chassis.moveToPoint(35, 37, 1000, {.forwards = false, .maxSpeed = 50});
    pros::delay(1800);
    intakeall(12000);
    pros::delay(1000); // commit
    intakeall(-12000);
    pros::delay(300);
    intakeall(12000);
    pros::delay(2500);
    intakeall(0);
    chassis.moveToPoint(22, 18.128, 1000);// og 20.5
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(25, 47, 2000, {.forwards = false, .maxSpeed = 50});
    pros::delay(10); 
        break;

        case 67: // pid tuning 
        alldsr(false, true);
        pros::delay(3000);
        // chassis.setPose(7.5, 6.75, 0);
        chassis.moveToPoint(7.5, 31.5, 2000);
        pros::delay(10000);
        alldsr(false, true);
        break;

        case 69:
        alldsr(false, true);
        break;

        case 70:
        chassis.setPose(0, 0, 0);
        // chassis.turnToHeading(90, 2000);
        // chassis.turnToHeading(180, 2000);
        // chassis.turnToHeading(0, 2000, {.direction = pAngularDirection::CCW_COUNTERCLOCKWISE});
        pros::delay(10);
        chassis.moveToPoint(0, 24, 10000, {.maxSpeed = 100});
        break;

        case 21: 
        // chassis.setPose(0, 0, 0);
        // pros::delay(10);
        // chassis.moveToPoint(0, 6, 1000, {.maxSpeed = 40});
        // chassis.waitUntilDone();
        // pros::delay(2000);
        // leftonlydsr(false, 0);
        // pros::delay(100000);

        chassis.setPose(0, 0, 0);
        lemleftdsr();

        chassis.moveToPoint(e2, 12, 3000, {.maxSpeed = 40});

        pros::delay(3000);

        lemleftdsr();

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

    if(master.get_digital_new_press(DIGITAL_LEFT)) {
        leftonlydsr(false, 0);
    }

    if(master.get_digital_new_press(DIGITAL_A)) {
        rightonlydsr(false, 0);
    }

    if(master.get_digital_new_press(DIGITAL_UP)) {
        lemleftdsr();
    }

    if(master.get_digital_new_press(DIGITAL_X)) {
        lemrightdsr();
    }


        // delay to save resources
    pros::delay(10);
    }
}