#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-18, 20, -16});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({12, -14, 15});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
/* pros::Motor dtmotor1(1);  
pros::Motor dtmotor2(1);
pros::Motor dtmotor3(1);
pros::Motor dtmotor4(1);
pros::Motor dtmotor5(1);
pros::Motor dtmotor6(1); */
pros::Motor intmotor1(1);
pros::Motor intmotor2(-2);
pros::Motor intmotor3(9);


/*
	void setIntakeHigh(int intakePower){
		intmotor1.move(intakePower);
		intmotor2.move(intakePower);
		intmotor3.move(intakePower);
}

	void setIntakeMid(int intakePower){
		intmotor1.move(intakePower);
		intmotor2.move(-intakePower);
		intmotor3.move(intakePower);
} */

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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage

		bool locktoggle = true;
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			locktoggle = !locktoggle;
		}

	// if (locktoggle = false){ // if button pressed
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (!locktoggle)){
			intmotor1.move_voltage(12000);
			intmotor2.move_voltage(12000);
			intmotor3.move_voltage(12000);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (!locktoggle)) {
			intmotor1.move_voltage(12000);
			intmotor2.move_voltage(12000);
			intmotor3.move_voltage(-12000);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (!locktoggle)) {
			intmotor1.move_voltage(-12000);
			intmotor2.move_voltage(-12000);
			intmotor3.move_voltage(-12000);
		} else {
			intmotor1.move_voltage(0);
			intmotor2.move_voltage(0);
			intmotor3.move_voltage(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (locktoggle)){
			intmotor1.move_voltage(12000);
			intmotor2.move_voltage(12000);
			intmotor3.move_voltage(0);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (locktoggle)) {
			intmotor1.move_voltage(12000);
			intmotor2.move_voltage(12000);
			intmotor3.move_voltage(0);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (locktoggle)) {
			intmotor1.move_voltage(-12000);
			intmotor2.move_voltage(-12000);
			intmotor3.move_voltage(0);
		} else {
			intmotor1.move_voltage(0);
			intmotor2.move_voltage(0);
			intmotor3.move_voltage(0); // stop moving
		}

		
		/* if(pros::E_CONTROLLER_DIGITAL_R1){
			setIntakeHigh(127);
		} else if (pros::E_CONTROLLER_DIGITAL_X) {
			setIntakeHigh(-127);
		} else {
			setIntakeHigh(0);
		}

		if(pros::E_CONTROLLER_DIGITAL_L1){
			setIntakeMid(127);
		} else if (pros::E_CONTROLLER_DIGITAL_X) {
			setIntakeHigh(-127);
		} else {
			setIntakeMid(0);
		} */

		pros::delay(20);                               // Run for 20 ms then update
	}
}