#include "declarations.hpp"
#include <cmath>

void rightdsr() {
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
sideSensorReading = sensor1.get(); //blhr
     opposite = c - a;
theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;

trackingCentreToWallSide = sideSensorReading + (width / 2); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

//  back of robot? 
distanceFromCentreBack = ((c + a) / 2) + (length / 2); // y distance from tracking center, length / 2 is tracking center
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack; // idek but its x 

centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;
distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4;
};

void leftdsr() {
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
sideSensorReading = sensor4.get();

     opposite = c - a;

theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;

trackingCentreToWallSide = sideSensorReading + (width / 2); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

//  back of robot? 
distanceFromCentreBack = ((c + a) / 2) + (length / 2); // y distance from tracking center, length / 2 is tracking center
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack; // idek but its x 

centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;
distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4;
};

void backdsr(){
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
    opposite = c - a;

theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;

distanceFromCentreBack = ((c + a) / 2) + (length / 2);
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack;

}

void rightonlydsr(bool override, double x){

if (override == true) {
    sideSensorReading = sensor1.get();

    theta = chassis.getPose().theta;

    trackingCentreToWallSide = sideSensorReading + (width / 2); 
    centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide;

    centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;

    centreToWallSideAccount4Angle = x - centreToWallSideAccount4Angle;

    chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, theta);
    
} else if (override == false) {
    sideSensorReading = sensor1.get();

    theta = chassis.getPose().theta;

    trackingCentreToWallSide = sideSensorReading + (width / 2); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
    centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

    centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;

    chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, theta);
} // i gotta work on abstraction

}

void leftonlydsr(bool override, double x){

if (override == true) {
    sideSensorReading = sensor4.get();

    theta = chassis.getPose().theta;

    trackingCentreToWallSide = sideSensorReading + (width / 2); 
    centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide;

    centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;

    centreToWallSideAccount4Angle = x - centreToWallSideAccount4Angle;

    chassis.setPose(centreToWallSideAccount4Angle, chassis.getPose().y, theta);

} else if (override == false) {
    sideSensorReading = sensor4.get();

    theta = chassis.getPose().theta;

    trackingCentreToWallSide = sideSensorReading + (width / 2); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
    centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

    centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;

    chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, theta);
} // i gotta work on abstraction

}

void lemrightdsr(){
width = -25.4;
length = -50.8;

c = sensor2.get();
a = sensor3.get();
sideSensorReading = sensor1.get(); //blhr
     opposite = c - a;
theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;

trackingCentreToWallSide = sideSensorReading + (width); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

//  back of robot? 
distanceFromCentreBack = ((c + a) / 2) + (length); // y distance from tracking center, length / 2 is tracking center
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack; // idek but its x 

centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;
distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4;

chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle,theta);

}

void lemleftdsr(){

width = -25.4;
length = -50.8;

c = sensor2.get();
a = sensor3.get();
sideSensorReading = sensor4.get(); //blhr
     opposite = c - a;
theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;

trackingCentreToWallSide = sideSensorReading + (width); // adjacent from tracking center to wall, sideSensorReading is from sensor to wall, width / 2 adds tracking center distance
centreToWallSideAccount4Angle = cos(theta * M_PI / 180.0) * trackingCentreToWallSide; // cos theta*hypotenuse = adjacent 

//  back of robot? 
distanceFromCentreBack = ((c + a) / 2) + (length); // y distance from tracking center, length / 2 is tracking center
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack; // idek but its x 

centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4;
distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4;

chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle,theta);

}

void alldsr(bool right, bool left){
    if (right == true){
        rightdsr();
        chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle,theta);
    }
    else if (left == true) {
        leftdsr();
        chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle,theta);
    }
    else {
        backdsr();
    }
}