#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"
#include "CAN/can_utils.hpp"
#include "Drive/SwerveModule.hpp"
#include "Drive/SwerveController.hpp"
#include "Joystick/mtJoystick.hpp"
#include "util/misc.hpp"

void usercontrol(){
    // Set up the joystick
    char* jsSource = "/dev/input/js0";
    bool joystickConnected = false;

    if (!startDeviceConnection(jsSource)) {
        printf("ERROR: joystick could not be initialized.\n");
    } else {
        printJoystickInformation();
        joystickConnected = true;
    }

    setCalibrationCoefficients(0, 0, 0, 0, 0, 255);


    // open the can connection
    CANConnection canConnection("can0");


    /*********************************************************************/
    /*                                                                   */
    /*                        Swerve Drive Init                          */
    /*                                                                   */
    /*********************************************************************/

    SparkMaxMC nwDrive(canConnection, 1);
    SparkMaxMC nwPivot(canConnection, 2);
    SparkMaxMC neDrive(canConnection, 3);
    SparkMaxMC nePivot(canConnection, 4);
    SparkMaxMC seDrive(canConnection, 5);
    SparkMaxMC sePivot(canConnection, 6);
    SparkMaxMC swDrive(canConnection, 7);
    SparkMaxMC swPivot(canConnection, 8);

    CANNetwork canNetwork(canConnection);
    canNetwork.addDevice(neDrive);
    canNetwork.addDevice(nePivot);
    canNetwork.addDevice(nwDrive);
    canNetwork.addDevice(nwPivot);
    canNetwork.addDevice(seDrive);
    canNetwork.addDevice(sePivot);
    canNetwork.addDevice(swDrive);
    canNetwork.addDevice(swPivot);

    SwerveController drive(neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot, false);

    canNetwork.startHearbeat();

    drive.importCalibration("config.txt");
    drive.setSensitivity(0.3);
    std::cout << "Calibration Set\n";

    short lx = 127;  // joystick values
    short ly = 127;
    short rx = 127;
    short ry = 127;
    while(1) {
        std::ifstream file(jsSource);  // make sure joystick is still connected
        if(file.good()) {
            file.close();
        } else {
            joystickConnected = false;
        }


        if(joystickConnected) {
            handleJoystickEvents();
            getAxisValue(0, &lx);
            getAxisValue(1, &ly);
            getAxisValue(3, &rx);
            getAxisValue(4, &ry);

            //double x = (lx - 127) / 127.0;
            //double y = (ly - 127) / -127.0;  // axis is reversed, multiply by -1
            //double w = (rx - 127) / 127.0;
            //drive.move(x, y, w);

            double x = lx/127;
            double y = ly/-127;
            double w = rx/(127*sqrt(2));
            drive.fixedMove(x, y, w);
            

        } else {
            drive.fixedMove(0, 0, 0);  // shut off drive

            // attempt to reconnect to joystick
            if (startDeviceConnection(jsSource)) {
                std::cout << "Joystick re-connected\n";
                printJoystickInformation();
                joystickConnected = true;
            }

        }

        sleep(5000);
    }

}

void tests() {
    /*********************************************************************/
    /*                                                                   */
    /*                Single Module Debug Section                        */
    /*                                                                   */
    /*********************************************************************/
    CANConnection canConnection("can0");

    SparkMaxMC motor1(canConnection, 1);
    SparkMaxMC motor2(canConnection, 2);

    motor1.dutyCycleSet(0);
    motor2.dutyCycleSet(0);

    motor1.setToFactoryDefaults();
    motor2.setToFactoryDefaults();
    motor2.setAltEncoderMode(false);

    motor1.setGearRatio(1/5.25);
    motor1.setTicksPerEncoderRevolution(4096);

    motor2.setGearRatio(1/(5.3333333 * 10));
    motor2.setTicksPerEncoderRevolution(42);

    motor1.setMotorReversed(true);
    motor2.setMotorReversed(true);

    motor1.burnFlash();
    motor2.burnFlash();

    
    CANNetwork canNetwork(canConnection);  // start the heartbeat signal
    canNetwork.addDevice(motor1);
    canNetwork.addDevice(motor2);
    canNetwork.startHearbeat();

    sleep(200000);  // wait a little bit for everything to start up

    motor1.tareEncoder();
    motor2.tareEncoder();

    SwerveModule s1(motor1, motor2);
    s1.setMountLocation(-0.5, 0.5);
    s1.setUsePWM(true);
    s1.setSensitivity(0.35);


    s1.moveSingular(0,0,0);

    sleep(20000);
    
    s1.moveSingular(0,1,0);

    // std::cout << "Starting calibration\n";
    // drive.calibrate("config.txt", 1000);
    // std::cout << "Calibration finished\n";

}

int main() {        
    tests();

    return 0;
}
