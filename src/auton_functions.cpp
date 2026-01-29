#include "declarations.hpp"
#include <cmath>

void rightdsr() {
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
d = sensor1.get();
     w = c - a;
theta = atan(w / b) * 180.0 / M_PI;

d2 = d + (width / 2); // adjacent from tracking center to wall, d is from sensor to wall, width / 2 adds tracking center distance
e2 = cos(theta * M_PI / 180.0) * d2; // cos theta*hypotenuse = adjacent 

//  back of robot? 
y2 = ((c + a) / 2) + (length / 2); // y distance from tracking center, length / 2 is tracking center
x2 = cos(theta * M_PI / 180.0) * y2; // idek but its x 

e2 = e2 / 25.4;
x2 = x2 / 25.4;
};

void leftdsr() {
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
d = sensor4.get();

     w = c - a;

theta = atan(w / b) * 180.0 / M_PI;

d2 = d + (width / 2); // adjacent from tracking center to wall, d is from sensor to wall, width / 2 adds tracking center distance
e2 = cos(theta * M_PI / 180.0) * d2; // cos theta*hypotenuse = adjacent 

//  back of robot? 
y2 = ((c + a) / 2) + (length / 2); // y distance from tracking center, length / 2 is tracking center
x2 = cos(theta * M_PI / 180.0) * y2; // idek but its x 

e2 = e2 / 25.4;
x2 = x2 / 25.4;
};

void backdsr(){
    //'*180.0/M_PI' converts radians to degrees
c = sensor2.get();
a = sensor3.get();
    w = c - a;

theta = atan(w / b) * 180.0 / M_PI;

y2 = ((c + a) / 2) + (length / 2);
x2 = cos(theta * M_PI / 180.0) * y2;

}

void alldsr(bool right, bool left){
    if (right == true){
        rightdsr();
        chassis.setPose(e2,x2,theta);
    }
    else if (left == true) {
        leftdsr();
        chassis.setPose(e2,x2,theta);
    }
    else {
        backdsr();
    }
}