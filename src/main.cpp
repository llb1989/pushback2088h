#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_mg({-18, 20, -16});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup left_mg({11, -14, 17});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::Motor intmotor1(1); // bottom roller
pros::Motor intmotor2(-2); // middle roller
pros::Motor intmotor3(9); // top 

pros::Imu imu(19); // make real port

pros::adi::Pneumatics littlewill('A', false);
pros::adi::Pneumatics fakeewill('B', false);

void intakeall(int intakepower, int intaketime) {
            intmotor1.move_voltage(intakepower);
            intmotor2.move_voltage(intakepower);
            intmotor3.move_voltage(intakepower);
            pros::delay(intaketime);
}
void intakeone(int intakepower, int intaketime) {
            intmotor1.move_voltage(intakepower);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0);
            pros::delay(intaketime);
}

bool locktoggle = false;
bool slowtoggle = false;

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings angular_controller(2,  // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15.7, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              500, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

    int autonumber = 1;
    switch (autonumber) {
// case 1: right red will wokr 100%%! 
//
    case 1:
    // set position to x:0, y:0, heading:0
    chassis.setPose(-161, -38, 0);
    pros::delay(67);
    // move 48" forwards
    chassis.moveToPoint(-113, -38, 1200);//go forwards to position to get balls
        pros::delay(67);
    intakeone(12000, 1000);
    chassis.moveToPoint(-59, -58, 1200); //grab group of 3 balls
    chassis.moveToPoint(-28, -27, 1200); // go score middle goal
    intakeone(-8000, 1000);//outaek
    intakeall(-8000, 500);//outake big
    chassis.moveToPoint(-120, -120, 1200); // go position matchload
    chassis.turnToHeading(180, 120); // position matchload
    littlewill.toggle();
    intakeone(12000, 2000); 
    chassis.moveToPoint(-150, -120, 1000); //amtchlodad nice
    chassis.moveToPoint(-76, -120, 1200); // score long goal
    intakeall(12000, 2000);
    break; 


    case 2:
    chassis.setPose(5, 0, 0);
    pros::delay(67);
    chassis.moveToPose(12, 53, 45, 6000);
    break;


    }


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


		
void opcontrol() {
	
	while (true) {
		
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme. 
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage	

	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // turn lock on and off
      locktoggle = !locktoggle; 
	}

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { // turn lock on and off
      slowtoggle = !slowtoggle; 
	}

    if (locktoggle && slowtoggle) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intmotor1.move_voltage(6000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intmotor1.move_voltage(6000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intmotor1.move_voltage(-6000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0); // stop moving
        }

    } else if (locktoggle) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intmotor1.move_voltage(12000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intmotor1.move_voltage(12000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intmotor1.move_voltage(-12000);
            intmotor2.move_voltage(0);
            intmotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
            intmotor3.move_voltage(0);
			intmotor3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // lock 
        } else {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0); // stop moving
        }

    } else if (slowtoggle) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intmotor1.move_voltage(3000);
            intmotor2.move_voltage(3000);
            intmotor3.move_voltage(6000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intmotor1.move_voltage(3000);
            intmotor2.move_voltage(3000);
            intmotor3.move_voltage(-6000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intmotor1.move_voltage(-3000);
            intmotor2.move_voltage(-3000);
            intmotor3.move_voltage(-6000);
        } else {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0); // stop moving
        }

    } else {
    
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intmotor1.move_voltage(12000);
            intmotor2.move_voltage(12000);
            intmotor3.move_voltage(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intmotor1.move_voltage(12000);
            intmotor2.move_voltage(12000);
            intmotor3.move_voltage(-12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intmotor1.move_voltage(-12000);
            intmotor2.move_voltage(-12000);
            intmotor3.move_voltage(-12000);
        } else {
            intmotor1.move_voltage(0);
            intmotor2.move_voltage(0);
            intmotor3.move_voltage(0); // stop moving
        }

    }

if(master.get_digital_new_press(DIGITAL_A)) {
    littlewill.toggle();
}

	}
		pros::delay(10);                               // Run for 10 ms then update
	}
