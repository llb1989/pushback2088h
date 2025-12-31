#include "declarations.hpp"

void intakeall(int intakepower) {
    midgoal.set_value(true);
    intmotor1.move_voltage(intakepower);
    intmotor3.move_voltage(intakepower);
        
}
void intakeone(int intakepower) {
    midgoal.set_value(true);
    intmotor1.move_voltage(intakepower);
    intmotor3.move_voltage(-intakepower + 8000);
 
}

void intakeback(int intakepower) {
    midgoal.set_value(true);
    intmotor1.move_voltage(0);
    intmotor3.move_voltage(intakepower);

}

void intakemiddle(int intakepower) {
    midgoal.set_value(false);
    intmotor1.move_voltage(intakepower);
    intmotor3.move_voltage(- intakepower);
} 

void forwards(int intakepower, int left) {
    rightMotors.move_voltage(intakepower);
    leftMotors.move_voltage(left);
}