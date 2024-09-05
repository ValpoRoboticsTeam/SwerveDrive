#include <fstream>
#include <string>
#include <iostream>

#include "CAN/SparkMaxMC.hpp"
#include "Drive/SwerveController.hpp"
#include "Drive/SwerveModule.hpp"
#include "VectorMath.hpp"
#include "util/misc.hpp"


SwerveController::SwerveController(
    SparkMaxMC& neDriveMotor,
    SparkMaxMC& nePivotMotor,
    SparkMaxMC& nwDriveMotor,
    SparkMaxMC& nwPivotMotor,
    SparkMaxMC& seDriveMotor,
    SparkMaxMC& sePivotMotor,
    SparkMaxMC& swDriveMotor,
    SparkMaxMC& swPivotMotor,
    bool useAltEncoder /* true */
) {
    neDrive = &neDriveMotor;
    nePivot = &nePivotMotor;
    nwDrive = &nwDriveMotor;
    nwPivot = &nwPivotMotor;
    seDrive = &seDriveMotor;
    sePivot = &sePivotMotor;
    swDrive = &swDriveMotor;
    swPivot = &swPivotMotor;

    // configure periodic data
    for(SparkMaxMC* motor : {neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot}) {
        motor->setToFactoryDefaults();
        motor->setPeriodicRate(0, 100);    // applied output, faults
        motor->setPeriodicRate(1, 5);      // velocity, temp, voltage, current
        motor->setPeriodicRate(2, 5);      // position data
        motor->setPeriodicRate(3, 65535);  // analog sensor - not used
        motor->setPeriodicRate(4, 5);      // alternate encoder
        motor->setPeriodicRate(5, 65535);  // duty cycle encoder - not used
        motor->setPeriodicRate(6, 65535);  // duty cycle encoder - not used
    }

    // configure drive motors
    neDrive->setMotorReversed(true);
    nwDrive->setMotorReversed(true);
    seDrive->setMotorReversed(true);
    swDrive->setMotorReversed(true);

    neDrive->setGearRatio(1/5.25);
    nwDrive->setGearRatio(1/5.25);
    seDrive->setGearRatio(1/5.25);
    swDrive->setGearRatio(1/5.25);

    neDrive->setTicksPerEncoderRevolution(4096);
    nwDrive->setTicksPerEncoderRevolution(4096);
    seDrive->setTicksPerEncoderRevolution(4096);
    swDrive->setTicksPerEncoderRevolution(4096);

    neDrive->setPIDF(1, 0, 0, 0);
    nwDrive->setPIDF(1, 0, 0, 0);
    seDrive->setPIDF(1, 0, 0, 0);
    swDrive->setPIDF(1, 0, 0, 0);
    

    // configure pivot motors
    nePivot->setMotorReversed(false);
    nwPivot->setMotorReversed(false);
    sePivot->setMotorReversed(false);
    swPivot->setMotorReversed(false);

    nePivot->setAltEncoderMode(useAltEncoder);
    nwPivot->setAltEncoderMode(useAltEncoder);
    sePivot->setAltEncoderMode(useAltEncoder);
    swPivot->setAltEncoderMode(useAltEncoder);


    nePivot->setAltEncoderReversed(true);
    nwPivot->setAltEncoderReversed(true);
    sePivot->setAltEncoderReversed(true);
    swPivot->setAltEncoderReversed(true);


    double ratio = useAltEncoder ? 1 : (1/(5.3333333 * 10));
    nePivot->setGearRatio(ratio);
    nwPivot->setGearRatio(ratio);
    sePivot->setGearRatio(ratio);
    swPivot->setGearRatio(ratio);

    int ticks = useAltEncoder ? 4096 : 42;
    nePivot->setTicksPerEncoderRevolution(ticks);
    nwPivot->setTicksPerEncoderRevolution(ticks);
    sePivot->setTicksPerEncoderRevolution(ticks);
    swPivot->setTicksPerEncoderRevolution(ticks);

    nePivot->setPIDF(1, 0, 0, 0);
    nwPivot->setPIDF(1, 0, 0, 0);
    sePivot->setPIDF(1, 0, 0, 0);
    swPivot->setPIDF(1, 0, 0, 0);
    

    // burn flash to save settings to motor controller in case of brown out
    for(SparkMaxMC* motor : {neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot}) {
        motor->burnFlash();
    }


    // configure swerve modules
    ne = new SwerveModule(*neDrive, *nePivot);
    nw = new SwerveModule(*nwDrive, *nwPivot);
    se = new SwerveModule(*seDrive, *sePivot);
    sw = new SwerveModule(*swDrive, *swPivot);

    ne->setUsePWM(true);
    nw->setUsePWM(true);
    se->setUsePWM(true);
    sw->setUsePWM(true);

    ne->setMountLocation(0.25, 0.25);
    nw->setMountLocation(-0.25, 0.25);
    se->setMountLocation(0.25, -0.25);
    sw->setMountLocation(-0.25, -0.25);

    ne->setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    nw->setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    se->setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    sw->setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
}


// frees memeory
SwerveController::~SwerveController() {
    delete ne;
    delete nw;
    delete se;
    delete sw;
}


// reads in a file
int SwerveController::importCalibration(const char calibrationConfigFile[255]) {
    std::ifstream file;           
    file.open(calibrationConfigFile); 

    std::string setting, op;
    float value;

    if(file.is_open()) {

        while(file >> setting >> op >> value) {
            if(setting == "neTare") {
                nePivot->setTarePosition(value);
            } else if(setting == "nwTare") {
                nwPivot->setTarePosition(value);
            } else if(setting == "seTare") {
                sePivot->setTarePosition(value);
            } else if(setting == "swTare") {
                swPivot->setTarePosition(value);
            }
        }

        file.close();

        return 0;
    } 

    return -1;
}


// samples every millisecond to take an average
int SwerveController::calibrate(const char calibrationConfigFile[255], int calibrationTime_ms) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = begin;

    int samples = 1;
    float nePivotSum = nePivot->getPosition();
    float nwPivotSum = nwPivot->getPosition();
    float sePivotSum = sePivot->getPosition();
    float swPivotSum = swPivot->getPosition();

    while(std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() < calibrationTime_ms) {
        nePivotSum += nePivot->getPosition();
        nwPivotSum += nwPivot->getPosition();
        sePivotSum += sePivot->getPosition();
        swPivotSum += swPivot->getPosition();

        samples++;
        sleep(1000);  // sleep for 1 ms
        end = std::chrono::steady_clock::now();
    }

    float neTare = ((nePivotSum / samples) / nePivot->getGearRatio()) + nePivot->getEncoderOffset();
    float nwTare = ((nwPivotSum / samples) / nwPivot->getGearRatio()) + nwPivot->getEncoderOffset();
    float seTare = ((sePivotSum / samples) / sePivot->getGearRatio()) + sePivot->getEncoderOffset();
    float swTare = ((swPivotSum / samples) / swPivot->getGearRatio()) + swPivot->getEncoderOffset();


    std::ofstream file(calibrationConfigFile);
    if(file.is_open()) {
        file << "neTare = " << neTare << "\n";
        file << "nwTare = " << nwTare << "\n";
        file << "seTare = " << seTare << "\n";
        file << "swTare = " << swTare << "\n";

        file.close();
        return 0;
    } else {
        return -1;
    }
}


// updates the sensitivity for each individual swerve module
void SwerveController::setSensitivity(int s) {
    sensitivity = s;
    ne->setSensitivity(s);
    nw->setSensitivity(s);
    se->setSensitivity(s);
    sw->setSensitivity(s);
}

void SwerveController::fixedMove(double inputX, double inputY, double w){
    switch(mode){
        case robot_centric:{
            // create the A, B, C, & D vector components to assign
            double A = inputX - w;
            double B = inputX + w;
            double C = inputY - w;
            double D = inputY + w;

            cartesian_vector NE = {B,C,0};
            cartesian_vector NW = {B,D,0};
            cartesian_vector SE = {A,C,0};
            cartesian_vector SW = {A,D,0};

            cartesian_vector unit = {1/sqrt(2),  1/sqrt(2),  0}; //vector of magnetude 1
            cartesian_vector max = copy(unit);
            if (greaterThan(NE,max)) {max =NE;}
            if (greaterThan(NW,max)) {max =NW;}
            if (greaterThan(SE,max)) {max =SE;}
            if (greaterThan(SW,max)) {max =SW;}

            if (greaterThan(max,unit)) { //if the max vector is larger than 1, scale all to 1.
                scale_vector(NE,max);
                scale_vector(NW,max);
                scale_vector(SE,max);
                scale_vector(SW,max);
            }

            /*
            ne->fixedMoveRobotCentric(NE, -M_PI / 2);  // offset by -pi/2 to accout for discrepancy in controller 0 angle and module 0 angle
            nw->fixedMoveRobotCentric(NW, -M_PI / 2);
            se->fixedMoveRobotCentric(SE, -M_PI / 2);
            sw->fixedMoveRobotCentric(SW, -M_PI / 2);
            */

            //turn each motor to their vector components
            ne->fixedMoveRobotCentric(inputX, inputY, w, NE);
            nw->fixedMoveRobotCentric(inputX, inputY, w, NW);
            se->fixedMoveRobotCentric(inputX, inputY, w, SE);
            sw->fixedMoveRobotCentric(inputX, inputY, w, SW);
 
            break;
        }
        case field_centric: {
            break;
        }
    }
}

void SwerveController::move(double inputX, double inputY, double w) {
    switch(mode) {
        case robot_centric: {
            ne->moveRobotCentric(inputX, inputY, w, -M_PI / 2);  // offset by -pi/2 to accout for discrepancy in controller 0 angle and module 0 angle
            nw->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            se->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            sw->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            break;
        }
        case field_centric: {

            break;
        }
    }
}
