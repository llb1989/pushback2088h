#include "declarations.hpp"

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