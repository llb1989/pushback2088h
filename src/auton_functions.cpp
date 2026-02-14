#include "declarations.hpp"
#include "main.h"
#include <cmath>
// #include <iterator>

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


// void setdsrpose(bool left, bool right, bool back){
//     if (left == true) {
//         leftdsr();
//         chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle, chassis.getPose().theta);
//     }
//     else if (right == true) {
//         rightdsr();
//         chassis.setPose(centreToWallSideAccount4Angle,distanceFromCentreBackAccount4Angle,chassis.getPose().theta);
//     }
//     else if (back == true) {
//         backdsr();
//         chassis.setPose(chassis.getPose().x,distanceFromCentreBackAccount4Angle,chassis.getPose().theta);
//     }
//     else {
//         chassis.setPose(chassis.getPose().x,chassis.getPose().y,theta); // never ever use DSR theta ever ever only imu theta 
//     }
// }

void lemreset(int setwall, int wall, bool left, bool right, bool back, double min_x, double max_x, double min_y, double max_y, bool setpose){ //1 is left
   
    if (setwall == true) {
        wall = wall;
    } else if (setwall == false) {
        if (chassis.getPose().theta >= 0 && chassis.getPose().theta < 90){
            wall = 1; // if heading = 0 reset right reset back
        } else if (chassis.getPose().theta >= 90 && chassis.getPose().theta < 180) {
            wall = 2; // if heading = 90 reset left - reset back?
        } else if (chassis.getPose().theta >= 180 && chassis.getPose().theta < 270) {
            wall = 3; // if heading = 180 reset left reset back 
        } else if (chassis.getPose().theta >= 270 && chassis.getPose().theta < 360) {
            wall = 4; // if heading = 270 reset right - reset back?
        }
    }
    
    switch(wall) {
        case 1:
            if (left == true) {
                leftdsr();
                centreToWallSideAccount4Angle = centreToWallSideAccount4Angle + min_x;
                if (setpose == true) {
                    chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, chassis.getPose().theta);
                }
            }
            if (right == true) {
                rightdsr();
                centreToWallSideAccount4Angle = max_x - centreToWallSideAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, chassis.getPose().theta);
                }
            }
            if (back == true) {
                backdsr();
                distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle + min_y;
                if (setpose == true) {
                    chassis.setPose(chassis.getPose().x,distanceFromCentreBackAccount4Angle, chassis.getPose().theta);
                }
            }

        break;

        case 2:
            if (left == true) {
                leftdsr();
                centreToWallSideAccount4Angle = max_y - centreToWallSideAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(chassis.getPose().x, centreToWallSideAccount4Angle, chassis.getPose().theta);
                }
            }
            if (right == true) {
                rightdsr();
                centreToWallSideAccount4Angle = min_y + centreToWallSideAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(chassis.getPose().x, centreToWallSideAccount4Angle, chassis.getPose().theta);
                }
            }
            if (back == true) {
                backdsr();
                distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle + min_x;
                if (setpose == true) {
                    chassis.setPose(distanceFromCentreBackAccount4Angle, chassis.getPose().y, chassis.getPose().theta);
                }
        }

        break;

        case 3:
            if (left == true) {
                    leftdsr();
                    centreToWallSideAccount4Angle = max_x - centreToWallSideAccount4Angle;
                    if (setpose == true) {
                        chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, chassis.getPose().theta);
                    }
                }
                if (right == true) {
                    rightdsr();
                    centreToWallSideAccount4Angle = min_x + centreToWallSideAccount4Angle;
                    if (setpose == true) {
                        chassis.setPose(centreToWallSideAccount4Angle,chassis.getPose().y, chassis.getPose().theta);
                    }
                }
                if (back == true) {
                    backdsr();
                    distanceFromCentreBackAccount4Angle = max_y - distanceFromCentreBackAccount4Angle;
                    if (setpose == true) {
                        chassis.setPose(chassis.getPose().x,distanceFromCentreBackAccount4Angle, chassis.getPose().theta);
                    }
            }

        break;

        case 4:
            if (left == true) {
                leftdsr();
                centreToWallSideAccount4Angle = min_y + centreToWallSideAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(chassis.getPose().x, centreToWallSideAccount4Angle, chassis.getPose().theta);
                }
            }
            if (right == true) {
                rightdsr();
                centreToWallSideAccount4Angle = max_y - centreToWallSideAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(chassis.getPose().x, centreToWallSideAccount4Angle, chassis.getPose().theta);
                }
            }
            if (back == true) {
                backdsr();
                distanceFromCentreBackAccount4Angle = max_x - distanceFromCentreBackAccount4Angle;
                if (setpose == true) {
                    chassis.setPose(distanceFromCentreBackAccount4Angle, chassis.getPose().y, chassis.getPose().theta);
                }
            }
        break;
    }                                                                   
}

// back dsr wall 1 = backdsr + min_y = y
// right dsr wall 1 = max_x - rightdsr = x
// left dsr wall 1 = ldsr + min_x = x

// back dsr wall 2 = backdsr + min_x = x
// right dsr wall 2 = min_y + rightdsr = y
// left dsr wall 2 = max_y - leftdsr = y

// back dsr wall 3 = max_y - backdsr = y
// right dsr wall 3 = min_x + rightdsr = x
// left dsr wall 3 = max_x - leftdsr = x

// back dsr wall 4 = max_y - backdsr = y
// right dsr wall 4 = max_y - rightdsr = y
// left dsr wall 4 = min_y + left = y


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

centreToWallSideAccount4Angle = centreToWallSideAccount4Angle / 25.4; // e2 = X
distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4; // x2 = Y
}

void lembackdsr() {
width = -25.4;
length = -50.8;

c = sensor2.get();
a = sensor3.get();

     opposite = c - a;
theta = atan(opposite / distanceBetweenBackSensors) * 180.0 / M_PI;
//  back of robot? 
distanceFromCentreBack = ((c + a) / 2) + (length); // y distance from tracking center, length / 2 is tracking center
distanceFromCentreBackAccount4Angle = cos(theta * M_PI / 180.0) * distanceFromCentreBack; // idek but its x 

distanceFromCentreBackAccount4Angle = distanceFromCentreBackAccount4Angle / 25.4; // x2 = y

}