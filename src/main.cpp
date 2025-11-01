#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-18, 20, -16}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup rightMotors({12, -14, 17}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::adi::Button autonselectbutton('C');

std::string job = "thanks";

//intake mototro
pros::Motor intmotor1(8); // first stage
pros::Motor intmotor2(-2); // middle roller
pros::Motor intmotor3(9); // top 

// Inertial Sensor on port 19
pros::Imu imu(19);

pros::adi::Pneumatics littlewill('A', false);
pros::adi::Pneumatics fakeewill('B', false);

    const int numAutos = 7;
//These are in  NOT degrees
int states[numAutos] = {0, 1, 2, 3, 4, 5, 6};
int currAuto = 0;

void nextState() {
    currAuto += 1;
    if (currAuto == numAutos) {
        currAuto = 1;
    }
}

void intakeall(int intakepower) {
            intmotor1.move_voltage(intakepower);
            intmotor2.move_voltage(intakepower);
            intmotor3.move_voltage(intakepower);
            // pros::delay(intaketime);
            // intmotor1.move_voltage(0);
            // intmotor2.move_voltage(0);
            // intmotor3.move_voltage(0);
}
void intakeone(int intakepower) {
            intmotor1.move_voltage(intakepower);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0);
            // pros::delay(intaketime);
            // intmotor1.move_voltage(0);
            // intmotor2.move_voltage(0);
            // intmotor3.move_voltage(0);
}

void intakeback(int intakepower) {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(intakepower);
            intmotor3.move_voltage(intakepower);
            // pros::delay(intaketime);
            // intmotor1.move_voltage(0);
            // intmotor2.move_voltage(0);
            // intmotor3.move_voltage(0);
}

void intakemiddle(int intakepower) {
    intmotor1.move_voltage(intakepower);
    intmotor2.move_voltage(intakepower);
    intmotor3.move_voltage(-intakepower);
}

void forwards(int intakepower, int left) {
    rightMotors.move_voltage(intakepower);
    leftMotors.move_voltage(left);
}

bool locktoggle = false;
bool slowtoggle = false;
// // tracking wheels
// // horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// // horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.58, // proportional gain (kP) meowigga
                                            0, // integral gain (kI)
                                            19.05, // derivative gain (kD) yayayayayayayay
                                            3, // anti windup NNNIIIII
                                            1, // small error range, in inchesGGG
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inchesEEERRR
                                            500, // large error range timeout, in milliseconds
                                            16 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.968, // proportional gain (kP) 
                                             0, // integral gain (kI) 
                                             13.17, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(
    // &vertical, // vertical tracking wheel
                            nullptr,
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // &horizontal, // horizontal tracking wheel
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
            } else {
                job = "no auto selected";
            }
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Auto: %d", currAuto);
            pros::lcd::print(4, "Auto name: %s", job);


            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() { // auto select
    while (true) {
        if (autonselectbutton.get_new_press()) {
    nextState();
        }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    nextState();
    }
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
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    int autonumber = currAuto;
    switch (autonumber) {
        // x,y,timeout WHY IS IT Y then X
        case 1: // right auto
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20.566, 1000); // forwards
    intakeone(12000);
    chassis.moveToPoint(8, 41.076, 1000, {.maxSpeed = 70});
    pros::delay(500);
    littlewill.toggle();
    pros::delay(600);
    intakeone(0);
    pros::delay(100);
    littlewill.toggle();
    chassis.turnToHeading(-45, 1000);
    chassis.moveToPoint(-6, 48, 1200, {.maxSpeed = 80});
    chassis.turnToHeading(-45, 1000);
     pros::delay(100);
    intakeall(-7500);
    pros::delay(200);
    intakeall(-4500);
    pros::delay(2000);
    chassis.moveToPoint(30, 17.665, 1500 , {.forwards = false ,.maxSpeed = 80});
    littlewill.toggle();
    chassis.turnToHeading(180, 1000); // move to matchload>
    chassis.moveToPoint(30, -6.5, 2000, {.maxSpeed = 30});
    intakeone(12000);
    chassis.moveToPoint(30, -6.5, 500, {.maxSpeed = 50});
    pros::delay(2500);
    intakeall(0);
    chassis.moveToPoint(31.5, 33, 1500, { .forwards = false ,.maxSpeed = 60});
    pros::delay(1000);
    intakeall(12000);
    pros::delay(1250);
    intakeall(0);
    break;

    case 2: 
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20.566, 1000);
    intakeone(12000);
    pros::delay(500);
    chassis.moveToPoint(-6.328, 40.076, 1000);
    pros::delay(250);
    littlewill.toggle();
    pros::delay(1500);
    littlewill.toggle();
    intakeone(0);
    chassis.moveToPoint(3, 52, 1000 , {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(225, 1000);
    pros::delay(400);
    intakemiddle(6500);
    pros::delay(3000);
    chassis.moveToPoint(-30, 17.665, 1500 , {.forwards = true ,.maxSpeed = 80}); // move to match
    littlewill.toggle();
    // ecds
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(-30, -2, 1000);
    intakeone(12000);
    pros::delay(1000);
    intakeall(0);
    chassis.moveToPoint(-30.5, 33, 1500, { .forwards = true ,.maxSpeed = 60});
    pros::delay(500);
    intakeall(12000);
    pros::delay(1250);
    intakeall(0);
    break;

    case 3:
    forwards(12000, 12000);
    pros::delay(500);
    forwards(0, 0); 
    break;
    
    case 4:
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(35, 15.292, 1500 , {.maxSpeed = 80});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(35, -1, 1000, {.maxSpeed = 80}); // move to matchload>
    pros::delay(1000);
    intakeall(0);
    chassis.moveToPoint(35.8, 33, 1200, { .forwards = false ,.maxSpeed = 70});
    pros::delay(1000);
    intakeall(12000);
    pros::delay(1600);
    intakeone(12000);
    chassis.moveToPoint(34, 16, 1500 , {.maxSpeed = 80}); // pull out?
    chassis.turnToHeading(-45, 1000);
    littlewill.toggle();
    chassis.moveToPoint(-2, 49.5, 2000, {.maxSpeed = 80});
    pros::delay(1000);
    intakeone(0);
    chassis.turnToHeading(-45, 1000);
     pros::delay(100);
    intakeone(-8500);
    pros::delay(600);
    intakeone(12000);
    chassis.moveToPoint(8, 41.076, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(-60, 20, 2000 , {.maxSpeed = 80});
    chassis.turnToHeading(180, 1000); // turn to matchload>
    littlewill.toggle();
    chassis.moveToPoint(-60, -5, 1500 , {.maxSpeed = 80});
    intakeone(12000);
    chassis.moveToPoint(-60, 33, 1500, { .forwards = false ,.maxSpeed = 70});
    pros::delay(400);
    intakeall(12000);
    break;

    case 5:
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(35, 15.292, 1500 , {.maxSpeed = 80});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(35, -1, 1000, {.maxSpeed = 50}); // move to matchload>
    pros::delay(500);
    intakeone(0);
    chassis.moveToPoint(35.8, 33, 1200, {.forwards = false ,.maxSpeed = 80});
    pros::delay(800);
    intakeall(12000);
    pros::delay(1400);
    intakeone(12000);
    chassis.moveToPoint(34, 16, 1500 , {.maxSpeed = 90}); // pull out?
    chassis.turnToHeading(-45, 1000);
    littlewill.toggle();
    chassis.moveToPoint(-2, 49.5, 2000, {.maxSpeed = 80});
    pros::delay(1000);
    intakeone(0);
    chassis.turnToHeading(-45, 800);
    pros::delay(100);
    intakeone(-8500);
    pros::delay(500);
    intakeone(12000); // score middle
    chassis.moveToPoint(8, 41.076, 1000, {.forwards = false, .maxSpeed = 80});
    // chassis.turnToHeading(-90, 800);
    chassis.moveToPoint(-60, 20, 1200 , {.maxSpeed = 100});
    // chassis.moveToPoint(-60, 10, 500, {.minSpeed = 60});
    chassis.turnToHeading(180, 1000); // turn to matchload
    intakeall(5000);
    chassis.moveToPoint(-55, 33, 1500, {.forwards = false ,.maxSpeed = 70});
    pros::delay(400);
    intakeall(12000);
    break;


    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    master.print(1, 1, "Auto mb: %d", currAuto);
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && (pros::E_CONTROLLER_DIGITAL_DOWN)){ // run auto
      autonomous();
	}

        if (autonselectbutton.get_new_press()) {
    nextState();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    nextState();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // turn lock on and off
      locktoggle = !locktoggle; 
	}

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { // turn lock on and off
      slowtoggle = !slowtoggle; 
	}

    if (locktoggle && slowtoggle) { 

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intakeone(5000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeone(5000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeone(-5000);
        } else {
            intakeall(0);
        }

    } else if (locktoggle) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intakeone(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeone(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeone(-12000);
        } else {
            intakeall(0);
        }

    } else if (slowtoggle) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intakeall(5000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakemiddle(5000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeall(-5000);
        } else {
            intakeall(0);
        }

    } else {
    
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intakeall(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakemiddle(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeall(-12000);
        } else {
            intakeall(0);
        }

    }

if(master.get_digital_new_press(DIGITAL_Y)) {
    littlewill.toggle();
}
        pros::delay(10);
    }
}