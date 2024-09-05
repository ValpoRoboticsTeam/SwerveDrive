#include <iostream>

#include "Drive/SwerveModule.hpp"
#include "util/misc.hpp"


// ctor - sets references to motors
SwerveModule::SwerveModule(SparkMaxMC& drive, SparkMaxMC& pivot) {
    driveMotor = &drive;
    pivotMotor = &pivot;

    pidf_constants constants = {2.3, 0, 0, 0, 0, 0, -1, 1};  // default constants
    controller.setConstants(constants);
    controller.setWrapAngle(true);  // we will use the controller for angles so allow wrap-around
}


// uses vector math to update the targets
void SwerveModule::moveToTarget(double inputX, double inputY, double w, double thetaOffset_rad) {
    cartesian_vector rotatedInputs = rotateVector({inputX, inputY, 0}, thetaOffset_rad);  // rotate the vector
    inputX = rotatedInputs.x;
    inputY = rotatedInputs.y;

    cartesian_vector strafeContrib = {inputX, inputY, 0.0};  // contribution to velocity from strafing vector

    cartesian_vector omega = {0.0, 0.0, -w};  // from RHR for clockwise to be positive z direction must be negative
    cartesian_vector position = {xPos_m, yPos_m, 0.0};
    cartesian_vector turnContrib = cross_product(omega, position);  // contribution to velocity from turning vector (w cross r)
    
    // normalize turn contrib vector
    // from cross product |x| = |(y1 * z2) - (z1 * y2)|
    // we are taking omega cross position, so y1 * z2 will always be 0, (no z contribution from 2d 
    // position vector) thus the max this term can be is omega=1 (from normalization) times the y 
    // position (yPos_m). 
    // A similar argument can be made for the maximum y value of this vector, which 
    // will just be the x position (xPos_m)
    scale(turnContrib.x, -1.0, 1.0, -yPos_m, yPos_m);
    scale(turnContrib.y, -1.0, 1.0, -xPos_m, xPos_m);

    //creation of the target vector!!!!
    cartesian_vector target = add_vectors(strafeContrib, turnContrib);  // add the contributions

    // ensure target vector has a maximum magnitude of 1, if not use the unit vector
    double targetVx = target.x;
    double targetVy = target.y;
    double m = magnitude({target.x, target.y, 0});
    if(m > 1) {  // if magnitude is outside range, use the unit vector instead
        targetVx = target.x / m;
        targetVy = target.y / m;
    }


    // calculate the angle to move to based on the current angle
    double currentAngle_rad = pivotMotor->getAngle_rad();
    if(currentAngle_rad > M_PI) currentAngle_rad -= 2 * M_PI;  // convert to [-pi, pi]
    
    double fTargetAngle_rad = atan2(targetVy, targetVx);         // angle between [-pi, pi] if motor were to move forwards
    double bTargetAngle_rad = atan2(targetVy, targetVx) + M_PI;  // angle between [-pi, pi] if motor were to move backwards
    if(bTargetAngle_rad > M_PI) bTargetAngle_rad -= 2 * M_PI;

    double targetAngle_rad;  // the closest angle to move to
    int sgn;                 // multiplier for if going backwards or backwards
    if(angleDiff_rad(currentAngle_rad, fTargetAngle_rad) <= angleDiff_rad(currentAngle_rad, bTargetAngle_rad)) {  // forwards is less change
        targetAngle_rad = fTargetAngle_rad;
        sgn = 1;
    } else {  // backwards is less change
        targetAngle_rad = bTargetAngle_rad;
        sgn = -1;
    }

    // update the target velocities factoring in the sign
    double driveV = sgn * sensitivity * magnitude({targetVx, targetVy, 0});

    if(controller.getSetpoint() != targetAngle_rad) {  // if new setpoint
        controller.newSetpoint(targetAngle_rad);
    }

    double pivotV = controller.step(currentAngle_rad);

    // std::cout << "Target Vx: " << targetVx << "    Target Vy: " << targetVy << "    Target angle: " << targetAngle_rad << "    pivotV " << pivotV << "\n";

    if(usePWM) {
        driveMotor->dutyCycleSet(driveV);
        pivotMotor->dutyCycleSet(pivotV);
    } else {
        driveMotor->velocitySet(driveV * maxDriveVelocity);
        pivotMotor->velocitySet(pivotV * maxPivotVelocity);
    }
}

void SwerveModule::fixedMoveToTarget(cartesian_vector target){
    
    //some method to turn the motor towards the given vector.
    // ensure target vector has a maximum magnitude of 1, if not use the unit vector
    double targetVx = target.x;
    double targetVy = target.y;
    double m = magnitude({target.x, target.y, 0});
    if(m > 1) {  // if magnitude is outside range, use the unit vector instead
        targetVx = target.x / m;
        targetVy = target.y / m;
    }


    // calculate the angle to move to based on the current angle
    double currentAngle_rad = pivotMotor->getAngle_rad();
    if(currentAngle_rad > M_PI) currentAngle_rad -= 2 * M_PI;  // convert to [-pi, pi]
    
    double fTargetAngle_rad = atan2(targetVy, targetVx);         // angle between [-pi, pi] if motor were to move forwards
    double bTargetAngle_rad = atan2(targetVy, targetVx) + M_PI;  // angle between [-pi, pi] if motor were to move backwards
    if(bTargetAngle_rad > M_PI) bTargetAngle_rad -= 2 * M_PI;

    double targetAngle_rad;  // the closest angle to move to
    int sgn;                 // multiplier for if going backwards or backwards
    if(angleDiff_rad(currentAngle_rad, fTargetAngle_rad) <= angleDiff_rad(currentAngle_rad, bTargetAngle_rad)) {  // forwards is less change
        targetAngle_rad = fTargetAngle_rad;
        sgn = 1;
    } else {  // backwards is less change
        targetAngle_rad = bTargetAngle_rad;
        sgn = -1;
    }

    // update the target velocities factoring in the sign
    double driveV = sgn * sensitivity * magnitude({targetVx, targetVy, 0});

    if(controller.getSetpoint() != targetAngle_rad) {  // if new setpoint
        controller.newSetpoint(targetAngle_rad);
    }

    double pivotV = controller.step(currentAngle_rad);

    // std::cout << "Target Vx: " << targetVx << "    Target Vy: " << targetVy << "    Target angle: " << targetAngle_rad << "    pivotV " << pivotV << "\n";

    if(usePWM) {
        driveMotor->dutyCycleSet(driveV);
        pivotMotor->dutyCycleSet(pivotV);
    } else {
        driveMotor->velocitySet(driveV * maxDriveVelocity);
        pivotMotor->velocitySet(pivotV * maxPivotVelocity);
    }
}


void SwerveModule::fixedMoveRobotCentric(double inputX, double inputY, double w, cartesian_vector target){
     
     // Set motion to 0 if inputs are all 0, otherwise it will still rotate wheels to 0 position
     // rather than not doing any motion
    if(inputX == 0 && inputY == 0 && w==0) { 
        if(usePWM) {
            driveMotor->dutyCycleSet(0);
            pivotMotor->dutyCycleSet(0);
        } else {
            driveMotor->velocitySet(0);
            pivotMotor->velocitySet(0);
        }
    } else {
        fixedMoveToTarget(target);
    }
}

// wrapper for moveToTarget
void SwerveModule::moveRobotCentric(double inputX, double inputY, double w, double thetaOffset_rad) {
     // Set motion to 0 if inputs are all 0, otherwise it will still rotate wheels to 0 position
     // rather than not doing any motion
    if(inputX == 0 && inputY == 0 && w==0) { 
        if(usePWM) {
            driveMotor->dutyCycleSet(0);
            pivotMotor->dutyCycleSet(0);
        } else {
            driveMotor->velocitySet(0);
            pivotMotor->velocitySet(0);
        }
    } else {
        moveToTarget(inputX, inputY, w, thetaOffset_rad);
    }
}

//singular debug wrapper for NE test module
void SwerveModule::moveSingular(double inputX, double inputY, double w){
    double A = inputX - w;
    double B = inputX + w;
    double C = inputY - w;
    double D = inputY + w;

    cartesian_vector target = {B,C,0};
    
    cartesian_vector unit = {1/sqrt(2),  1/sqrt(2),  0}; //vector of magnetude 1
    cartesian_vector max = copy(unit);

    if (greaterThan(target,max)) {max=target;}
    if (greaterThan(max,unit)) { //if the max vector is larger than 1, scale all to 1.
        scale_vector(target,max);
    }

    fixedMoveRobotCentric(inputX, inputY, w, target);
}