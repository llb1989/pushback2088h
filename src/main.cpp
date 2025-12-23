#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
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

std::string job = "thanks";

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

    const int numAutos = 9;
int states[numAutos] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
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
        
}
void intakeone(int intakepower) {
            intmotor1.move_voltage(intakepower);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0);
 
}

void intakeback(int intakepower) {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(intakepower);
            intmotor3.move_voltage(intakepower);

}

void intakemiddle(int intakepower) {
    intmotor1.move_voltage(intakepower);
    intmotor2.move_voltage(-intakepower);
    intmotor3.move_voltage(intakepower);
}

void intakefreaky(int intakepower) {
    intmotor1.move_voltage(12000);
    intmotor2.move_voltage(12000);
    intmotor3.move_voltage(-intakepower);
}

void forwards(int intakepower, int left) {
    rightMotors.move_voltage(intakepower);
    leftMotors.move_voltage(left);
}

bool locktoggle = false;
bool slowtoggle = false;


 pros::Distance sensor1(5); // right side
 pros::Distance sensor2(6); //  right side middle
 pros::Distance sensor3(7); // left side middle
 pros::Distance sensor4(8); // left side

int d = 1;
int c = 1;
int a = 1;
int w = 1;
int b = 5;
int d2 = 1;
int e2 = 1;
int y2 = 1;
int x2 = 1;
int width = 67;
int length = 67;
int theta = 67;




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


void distancesensorresetright() {
sensor1.get();
sensor2.get();
sensor3.get();
c = sensor2.get();
a = sensor3.get();
d = sensor1.get();
if (c > a) {
    w = c - a;
}
else {
    w = a - c;
}
tan(w/b);

d2 = d + width/2;
e2 = cos(theta) * d2;
y2 = ((c + a) / 2) + length / 2;
x2 = cos(theta) * y2;

chassis.setPose(e2,x2,theta);
};

void distancesensorresetleft() {
sensor4.get();
sensor2.get();
sensor3.get();
c = sensor2.get();
a = sensor3.get();
d = sensor4.get();
if (c > a) {
    w = c - a;
}
else {
    w = a - c;
}
tan(w/b);

d2 = d + width/2;
e2 = cos(theta) * d2;
y2 = ((c + a) / 2) + length / 2;
x2 = cos(theta) * y2;

chassis.setPose(e2,x2,theta);
};


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

        // print measurements from the rotation sensor
        pros::lcd::print(6, "Rotation Sensor: %i", rotation.get_position());
        pros::lcd::print(7, "Temp: %0.1f", drivetrainTemp);
        // make double avg motor temp
       
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
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Auto: %d", currAuto);
            pros::lcd::print(4, "Auto name: %s", job);
            // master.print(1, 2, "Auto: %d", currAuto);
            master.print(1, 2, "Y: %f", chassis.getPose().y);
            //master.print(1, 2, "Auto?: %s", job);


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
    // master.print(1, 2, "Auto: %f", currAuto);
    master.print(1, 3, "Y: %f", chassis.getPose().y);
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
    switch (15) {

        case 21: // forwards
        forwards(8000, 8000);
        pros::delay(100);
        forwards(0, 0);
        break;

        case 41: // left? 
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

         break;

        case 10:

        chassis.setPose(0,0,0);
        chassis.moveToPoint(0, 24, 2000);
        // chassis.setPose(0,0,0);
        // chassis.moveToPoint(-2, 26, 1000);
        // intakeone(8000);
        // chassis.turnToHeading(40, 1000, {.maxSpeed = 50});
        // chassis.moveToPoint(12, 38, 2000, {.maxSpeed = 40});
        // // littlewill.toggle();
        // pros::delay(1000);
        // chassis.turnToHeading(-45, 1000);
        // // littlewill.toggle();
        // chassis.moveToPoint(-12, 54, 1000, {.maxSpeed = 40});
        // intakeone(-12000);
        break;

        case 11: 
         chassis.setPose(0,0,0);
         chassis.moveToPoint(0, 32, 2000, {.maxSpeed = 60});
         pros::delay(10);
         chassis.turnToHeading(90, 1000);
         littlewill.toggle();
         intakeone(12000);
         pros::delay(10);
         chassis.moveToPoint(12, 32,1000);
         pros::delay(1000);
         intakeall(0);
        //  pros::delay(100);
        //  chassis.moveToPoint(-24, 33.5, 1000, {.forwards = false, .maxSpeed = 60});
        //  pros::delay(900);
        //  intakeall(12000);
        //  littlewill.toggle();
         break;

         case 12: 
         chassis.setPose(0,0,0);
         chassis.moveToPoint(0, 30, 2000, {.maxSpeed = 60});
         pros::delay(10);
         chassis.turnToHeading(-90, 1000);
         littlewill.toggle();
         intakeone(12000);
         chassis.moveToPoint(-18, 29,1000, {.forwards = true, .maxSpeed = 50});
         pros::delay(1000);
         chassis.moveToPoint(-20, 20,1000, {.forwards = true, .maxSpeed = 50});
         pros::delay(2000);
          intakeone(1000);

        chassis.moveToPoint(-6, 30, 1000,{.forwards = false, .maxSpeed = 50});
         intakeall(0);
        littlewill.toggle();
        chassis.turnToHeading(-215, 1000);
        intakeone(12000);
        chassis.moveToPoint(30, 0, 2000, {.forwards = true, .maxSpeed = 40});
        pros::delay(1000);
        littlewill.toggle();

        chassis.moveToPoint(0, 30, 1350, {.forwards = false, .maxSpeed = 60});
        chassis.turnToHeading(-90, 1000);

        chassis.moveToPoint(24, 30, 1000, {.forwards = false, .maxSpeed = 50});
        chassis.turnToHeading(-90, 1000);
        pros::delay(1000);
        chassis.turnToHeading(-90, 1000);
        intakeall(12000);
        //  chassis.moveToPoint(24, 30, 1000, {.forwards = false, .maxSpeed = 60});
        //  pros::delay(900);
        //  intakeall(12000);
        //  littlewill.toggle();
         break;

        case 1: // right auto
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20.566, 1000); // forwards
    intakeone(7000);
    chassis.moveToPoint(7.5, 41.076, 1000, {.maxSpeed = 70});
    pros::delay(600);
    littlewill.toggle();
    pros::delay(600);
    intakeone(12000);
    littlewill.toggle();
    chassis.turnToHeading(-43, 1000);
    chassis.moveToPoint(-9, 52, 1200, {.maxSpeed = 60});
    pros::delay(700);                                   
    intakeone(0);
    chassis.turnToHeading(-45, 1000);
    pros::delay(100);
    intakeall(-12000);
    pros::delay(400);
    intakeall(-12000);
    pros::delay(800);                                                                                                                                                                                                                                                                                                                
    intakeall(0);
    chassis.moveToPoint(30.5, 17.665, 1500 , {.forwards = false ,.maxSpeed = 80});
    littlewill.toggle();
    chassis.turnToHeading(180, 1000); // move to matchload
    chassis.moveToPoint(30, -7.5, 900, {.maxSpeed = 60});
    intakeone(12000);
    chassis.moveToPoint(30.6, -7.5, 200, {.maxSpeed = 80});
    pros::delay(300);
    intakeall(0);

    chassis.turnToHeading(180, 900);
    chassis.moveToPoint(31, 32, 1200, {.forwards = false ,.maxSpeed = 80});
    pros::delay(900);
    intakeall(12000);
    pros::delay(800);  
    intakeall(-12000);
    pros::delay(100);

    chassis.moveToPoint(32, 33, 1200, {.forwards = false ,.maxSpeed = 80}); 
    intakeall(12000);
    pros::delay(1600); 
    chassis.cancelAllMotions();
    forwards(-10000,-10000);
    break;

    case 2: // left auto
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20.566, 1000); // forward
    intakeone(12000);
    chassis.moveToPoint(-8, 42, 1000);  // move to blocks
    pros::delay(300);
    littlewill.toggle(); // down
    intakeone(12000);
    chassis.moveToPoint(5.5, 47.5, 1000 , {.forwards = false, .maxSpeed = 80}); // back into goal?
    chassis.turnToHeading(230, 1000);

    pros::delay(1500);
    intakemiddle(5500);
    pros::delay(1500);

    intakeone(12000);
    chassis.moveToPoint(-31.5, 17.665, 1500 , {.forwards = true ,.maxSpeed = 67}); // move to match
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-31.5, -7, 1000);
    pros::delay(1100);
    chassis.moveToPoint(-31.5, 32, 1500, {.forwards = false ,.maxSpeed = 60}); // goal?
    pros::delay(1000);
    intakeall(12000);
    chassis.moveToPoint(-31.5, 34, 1500, {.forwards = false ,.maxSpeed = 60}); // goal?
    intakeall(12000);
    pros::delay(2000);
    intakeall(0);
    break;

    case 3: // forwards
    forwards(12000, 12000);
    pros::delay(200);
    forwards(-500, -500);
    pros::delay(50);
    forwards(0, 0); 
    break;
    
    case 4: // sawp
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(34.5, 15.292, 1500 , {.maxSpeed = 80});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(34.5, -5, 1000, {.maxSpeed = 80}); // move to matchload>
    pros::delay(1150);

    chassis.moveToPoint(36.5, 34, 1200, { .forwards = false ,.maxSpeed = 70});
    pros::delay(550);
    // intakeall(0);
    // pros::delay(400);
    intakeall(12000);
    pros::delay(1400);
    intakeone(12000);
    chassis.moveToPoint(34, 16, 1400 , {.maxSpeed = 80}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();
    chassis.moveToPoint(-2, 49.5, 2000, {.maxSpeed = 90});
    pros::delay(550);
    intakeone(0);
    chassis.turnToHeading(-45, 500);
    intakeone(-12000);
    pros::delay(800);
    intakeone(12000);
    chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-90, 1000);


    chassis.moveToPoint(-38, 35, 1000 , {.maxSpeed = 80});
    pros::delay(700);
    littlewill.toggle();

    chassis.turnToHeading(227, 500);
    chassis.moveToPoint(-21, 47.5, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(1000);
    intakeone(0);
    intakefreaky(4000);
    pros::delay(50000);

    break;

    case 5: // idek what this is - maybe a BAD SAWP
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(34.5, 15.292, 1500 , {.maxSpeed = 80});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(34.5, -5, 1000, {.maxSpeed = 80}); // move to matchload>
    pros::delay(2000);

    chassis.moveToPoint(36.5, 34, 1200, { .forwards = false ,.maxSpeed = 70});
    pros::delay(1050);
    // intakeall(0);
    // pros::delay(400);
    intakeall(12000);
    pros::delay(2500);
    intakeone(12000);
    chassis.moveToPoint(34, 16, 1400 , {.maxSpeed = 80}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();
    chassis.moveToPoint(-2.5, 49.5, 2000, {.maxSpeed = 90});
    pros::delay(550);
    intakeone(0);
    chassis.turnToHeading(-45, 500);
    intakeone(-12000);
    pros::delay(2000);
    intakeone(12000);
    chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-90, 1000);


    chassis.moveToPoint(-38, 35, 1000 , {.maxSpeed = 80});
    pros::delay(1000);
    littlewill.toggle();

    chassis.turnToHeading(225, 500);
    chassis.moveToPoint(-20, 47, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(1000);
    intakeone(0);
    intakefreaky(4000);
    pros::delay(3000);
    
    chassis.moveToPoint(-38, 35, 1000 , {.forwards = false, .maxSpeed = 80});
    littlewill.toggle();
    chassis.moveToPoint(-40, -2, 2000, {.forwards = true, .maxSpeed = 80});
    chassis.turnToHeading(90, 1000);
    pros::delay(1000);
    chassis.moveToPoint(-20, -4, 2000, {.forwards = true, .maxSpeed = 80});
    pros::delay(1000);
    chassis.cancelAllMotions(); //neckhurt :()
    forwards(12000,12000);
    pros::delay(1000);
    forwards(0,0);
    break;

    case 6: // path
    
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(35.5, 15.292, 1500 , {.maxSpeed = 70});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(35.5, -2, 1000, {.maxSpeed = 80}); // move to matchload
    pros::delay(1050);
    chassis.moveToPoint(36, -2.2, 1000, {.maxSpeed = 80}); // move to matchload
    pros::delay(2000);

    chassis.moveToPoint(34, 16, 1500 , {.forwards = false, .maxSpeed = 60}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();

     pros::delay(1000);

    chassis.moveToPoint(-2, 49.5, 800, {.maxSpeed = 60});
    pros::delay(850);
    // intakeone(0);
    // chassis.turnToHeading(-45, 1000);
    //  pros::delay(100);
    // intakeone(-8500);
    // pros::delay(600);
    // intakeone(12000);
    
    
    
    pros::delay(1000);
    
    littlewill.toggle();
    intakeone(6000);
    chassis.moveToPoint(39, 16, 1500 , {.forwards = false, .maxSpeed = 60}); // pull out?
    chassis.turnToHeading(190, 900); // turn to matchload>


    // chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 60});
    // chassis.turnToHeading(-90, 1000);


    chassis.moveToPoint(39, 34, 1200, { .forwards = false ,.maxSpeed = 70});
    chassis.turnToHeading(190, 900); 
    pros::delay(300);
    // intakeall(0);
    // pros::delay(600);
    intakeall(12000);
    pros::delay(5200);
    intakeall(12000);
    pros::delay(1200);
    intakeone(12000);

    chassis.moveToPoint(38.5, 20, 1200, { .forwards = true ,.maxSpeed = 70});
    chassis.turnToHeading(188, 900); 
    chassis.moveToPoint(39, 36, 1200, { .forwards = false ,.maxSpeed = 120, .minSpeed = 60});

    pros::delay(1000);

    chassis.moveToPoint(34, 26, 1500 , {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(270, 900); 

    chassis.moveToPoint(-34, 30, 3000 , {.forwards = true, .maxSpeed = 70});

    break;

    case 7: // 67 path
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(35.5, 15.292, 1500 , {.maxSpeed = 80});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 900); // turn to matchload>
    chassis.moveToPoint(35, -6, 1000, {.maxSpeed = 90}); // move to matchload>
    intakeone(12000);
    pros::delay(1000);
    chassis.moveToPoint(35, -6, 1000, {.maxSpeed = 80}); // move to matchload>
    chassis.turnToHeading(180, 900); // turn to matchload>
    pros::delay(2000);

    
    pros::delay(1000);
    intakeone(12000);
    chassis.moveToPoint(36, 16, 1200, {.forwards = false ,.maxSpeed = 60});
    chassis.turnToHeading(270, 1200);

    chassis.moveToPoint(56, 16,1200, {.forwards = false ,.maxSpeed = 60});
    chassis.turnToHeading(180, 1200);
    chassis.moveToPoint(56, 100, 3000, {.forwards = false ,.maxSpeed = 50});

    chassis.turnToHeading(270, 1200);
    chassis.moveToPoint(42, 100, 1200, {.forwards = true,.maxSpeed = 50});
    chassis.turnToHeading(360, 1200);
    chassis.moveToPoint(42, 80, 1200, {.forwards = false ,.maxSpeed = 60});
    chassis.turnToHeading(365, 1200);
    intakeall(0);
    pros::delay(500);
    intakeall(12000);
    pros::delay(3000);
    intakeone(10000);
    chassis.moveToPoint(42, 118, 1200, {.forwards = true, .maxSpeed = 80});  
    chassis.turnToHeading(360, 1200);
    pros::delay(1000);
      chassis.moveToPoint(42.5, 120, 1200, {.forwards = true, .maxSpeed = 90}); 
      pros::delay(1000);
      chassis.moveToPoint(42, 124, 1200, {.forwards = true, .maxSpeed = 100, .minSpeed = 50}); 
      pros::delay(2000);
    chassis.turnToHeading(360, 1200); // back to goal
    chassis.moveToPoint(42.5, 80, 3000, {.forwards = false ,.maxSpeed = 60}); 
    intakeall(0);
    pros::delay(600);
    intakeall(12000);
    pros::delay(4000);
    littlewill.toggle();
    chassis.moveToPoint(24, 124, 2000, {.forwards = true,.maxSpeed = 60}); 
    chassis.turnToHeading(180, 900);

    chassis.moveToPoint(24, -10, 4000, {.forwards = true,.maxSpeed = 70});
    chassis.turnToHeading(250, 1000); 
    chassis.moveToPoint(20, -20, 2000, {.forwards = true,.maxSpeed = 70}); 
    intakeone(12000);
    chassis.cancelAllMotions();
    pros::delay(1000);
    intakeall(-12000);
    forwards(12000, 12000);
    pros::delay(1000);
    forwards(0, 0);
    
    break;

    case 8: // sawp
    chassis.setPose(-8, 15, 90);
    chassis.moveToPoint(35, 15.292, 1000 , {.maxSpeed = 90});
    littlewill.toggle();
    intakeone(12000);
    chassis.turnToHeading(180, 1000); // turn to matchload>
    chassis.moveToPoint(34, -3, 1000, {.maxSpeed = 90}); // move to matchload>
    pros::delay(1050);

    chassis.moveToPoint(34, 33, 1200, { .forwards = false ,.maxSpeed = 90});
    intakeall(12000);
    pros::delay(2100);
    intakeone(12000);
    chassis.moveToPoint(34, 16, 1500 , {.maxSpeed = 90}); // pull out?
    chassis.turnToHeading(-45, 500);
    littlewill.toggle();
    chassis.moveToPoint(-2.5, 50, 500, {.maxSpeed = 90});
    pros::delay(1000);
    chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-90, 1000);

    chassis.moveToPoint(-40, 36, 1000 , {.forwards = true, .maxSpeed = 90});
    pros::delay(500);
    littlewill.toggle();

    chassis.turnToHeading(225, 500);
    chassis.moveToPoint(-21.8, 46, 2000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1000);
    intakeone(0);
    chassis.turnToHeading(225, 200);
    pros::delay(100);
    intakemiddle(7000);
    pros::delay(500);
    intakeone(0);
    pros::delay(100);
    intakeone(10000);


    chassis.moveToPoint(-57, 5, 1200, {.forwards = true, .maxSpeed = 90});
    chassis.turnToHeading(180, 800);
    intakeone(12000);
    chassis.moveToPoint(-57, 30, 1200, {.forwards = false, .maxSpeed = 90});
    pros::delay(100);
    intakeall(12000);
    pros::delay(2000);

    break;

        case 9: // true sawp
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
    // chassis.setPose(-8, 15, 90);
    // chassis.moveToPoint(37, 15.292, 1000 , {.maxSpeed = 90});
    // littlewill.toggle();
    // intakeone(12000);
    // chassis.turnToHeading(180, 900); // turn to matchload>
    // chassis.moveToPoint(35.5, -2, 1000, {.maxSpeed = 90}); // move to matchload>
    // pros::delay(1050);

    // chassis.moveToPoint(36, 33, 1200, { .forwards = false ,.maxSpeed = 90});
    // intakeall(12000);
    // pros::delay(2100);
    // intakeone(12000);
    // chassis.moveToPoint(34, 16, 1500 , {.maxSpeed = 90}); // pull out?
    // chassis.turnToHeading(-45, 500);
    // littlewill.toggle();
    // chassis.moveToPoint(-2.5, 50, 500, {.maxSpeed = 90});
    // pros::delay(1200);
    // chassis.moveToPoint(8, 35, 1000, {.forwards = false, .maxSpeed = 90});
    // chassis.turnToHeading(-90, 1000);

    // chassis.moveToPoint(-40, 36, 1000 , {.forwards = true, .maxSpeed = 90});
    // pros::delay(500);
    // littlewill.toggle();

    // chassis.turnToHeading(225, 500);
    // chassis.moveToPoint(-21.8, 46, 2000, {.forwards = false, .maxSpeed = 90});
    // pros::delay(1000);
    // intakeone(0);
    // chassis.turnToHeading(225, 200);
    // pros::delay(100);
    // intakemiddle(7000);
    // pros::delay(500);
    // intakeone(0);
    // pros::delay(100);
    // intakeone(10000);


    // chassis.moveToPoint(-57, 5, 1200, {.forwards = true, .maxSpeed = 90});
    // chassis.turnToHeading(180, 800);
    // intakeone(12000);
    // // chassis.moveToPoint(-57, -6.5, 1000, {.forwards = true, .maxSpeed = 90}); // matchload ?
    // // pros::delay(600);
    // // intakeone(0);
    // // chassis.turnToHeading(180, 800);
    // chassis.moveToPoint(-56, 30, 1200, {.forwards = false, .maxSpeed = 90});
    // pros::delay(100);
    // intakeall(12000);
    // pros::delay(2000);

    break;
    
    case 15: // 7+ wing
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

     //   pros::delay(1000);
//     chassis.cancelAllMotions();
//     forwards(6000, -6000);
//     pros::delay(100);
//   forwards(0, 0);


break;

case 2088: // skills
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
     chassis.moveToPoint(30.5, 0, 500, {.maxSpeed = 60});
     pros::delay(1000);
     chassis.moveToPoint(30.8, -2, 500, {.maxSpeed = 60});
     pros::delay(1000);
    chassis.moveToPoint(31, 32, 1000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1200);
    chassis.moveToPoint(31.1, 34, 1000, {.forwards = false, .maxSpeed = 90});
    intakeall(12000);
    pros::delay(1000);
    intakeall(-12000);
    pros::delay(300);
    intakeall(12000);
    pros::delay(2000);
    intakeall(0);
    chassis.moveToPoint(20.7, 20.128, 1000);// og 20.5
    chassis.turnToHeading(-180, 500);
    chassis.moveToPoint(20.7, 44, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.moveToPoint(20, 20, 1000);

    chassis.turnToPoint(0, -8, 1000);
    chassis.moveToPoint(0, -8, 1000, {.forwards = true, .minSpeed = 70});

break;

    case 67: // pid tuning 
        chassis.setPose(0,0,0);
        chassis.moveToPoint(0, 24, 1500);
    break;
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // master.print(1, 2, "Auto: %f", currAuto);
    master.print(1, 3, "Y: %f", chassis.getPose().y);
    // loop to continuously update motors
    while (true) {
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

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { // turn lock on and off
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

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            chickenstars.toggle();
        }

    if(master.get_digital_new_press(DIGITAL_Y)) {
        littlewill.toggle();
    }
    pros::delay(10);
    }
}