// SwerveModule.hpp
// Contains class for controlling a swerve module consisting of 
// two sparkmax motors
//
// Author: Aiden Carney

#ifndef SWERVEMODULE_HPP
#define SWERVEMODULE_HPP

#include <cmath>

#include "CAN/SparkMaxMC.hpp"
#include "util/PIDController.hpp"
#include "util/VectorMath.hpp"


// Uses clockwise as positive w as convention
class SwerveModule {
    private:        
        double xPos_m;  // width of robot in meters
        double yPos_m;  // length of robot in meters

        SparkMaxMC* driveMotor;
        SparkMaxMC* pivotMotor;  // NOTE: this motor should be set up so that a positive signal moves it CCW

        int maxDriveVelocity = 5800;
        int maxPivotVelocity = 11000;
        double sensitivity = 1.0;
        bool usePWM = false;

        PIDController controller;  // the position pid controller for the pivot motor

        // updates the motor motion for each degree of freedom on the module
        // based on the inputs. Calculates where to pivot to and how fast
        // the drive motor should be spinning. The magnitude of x and y should
        // be 1
        //
        // Params:
        //    componentX      - the X component of the motor's movement vector
        //    componentY      - the X component of the motor's movement vector
        //
        //    thetaOffset_rad - an angle to rotate the input vector by. This is useful
        //                      because the of the discrepancy in where the 0 mark is for
        //                      the controller vs the 0 mark for input from a joystick
        // Return:
        //    None
        void fixedMoveToTarget(cartesian_vector target);

        // updates the motor motion for each degree of freedom on the module
        // based on the inputs. Calculates where to pivot to and how fast
        // the drive motor should be spinning. The magnitude of x and y should
        // be 1
        //
        // Params:
        //    inputX          - the target x direction, normalised to [-1, 1]
        //    inputY          - the target y direction, normalised to [-1, 1]
        //    w               - the target radial velocity, normalised to [-1, 1] where 1
        //                      is move full clockwise
        //    thetaOffset_rad - an angle to rotate the input vector by. This is useful
        //                      because the of the discrepancy in where the 0 mark is for
        //                      the controller vs the 0 mark for input from a joystick
        // Return:
        //    None
        void moveToTarget(double inputX, double inputY, double w, double thetaOffset_rad);


    public:

        // Ctor - sets the two motors for the swerve module
        // 
        // Params:
        //    drive - the drive motor on the module
        //    pivot - the pivot motor on the module
        // Return:
        //    the new instance
        SwerveModule(SparkMaxMC& drive, SparkMaxMC& pivot);


        // updates the position of the swerve module relative
        // to the center of the robot. The center is considered
        // (0, 0) and all measurements are in meters
        //
        // Params:
        //    x - the x position in meters
        //    y - the y position in meters
        // Return:
        //    None
        void setMountLocation(double x, double y) {
            xPos_m = x;
            yPos_m = y;
        }


        // sets the maximum rpm for each motor in the serve module,
        // this is necessary for the velocity controllers
        //
        // Params:
        //    driveRPM - the max RPM for the drive motor
        //    pivotRPM - the max RPM for the pivot motor
        // Return:
        //    None
        void setMaxMotorRPM(int driveRPM, int pivotRPM) {
            maxDriveVelocity = driveRPM;
            maxPivotVelocity = pivotRPM;
        }


        // Sets a constant to multiplied by the velocity output, should be
        // on interval [0, 1]
        //
        // Params:
        //    newSensitivity - the new constant to multiply the output by
        // Return:
        //    None
        void setSensitivity(double newSensitivity) {
            sensitivity = newSensitivity;
        }


        // Sets whether motor motion commands should be PWM commands or use
        // the built-in velocity controller. The built-in motor velocity 
        // controller should only be used if its PID constants are used. Motors
        // will always use the slot 0 pwm constants
        //
        // Params:
        //    pwm - use pwm control or velocity control
        // Return:
        //    None
        void setUsePWM(bool pwm) {
            usePWM = pwm;
        }


        // Sets new PID constants for the pivot position controller
        //
        // Params:
        //    constants - the new constants
        // Return:
        //    None
        void setPIDConstants(pidf_constants constants) {
            controller.setConstants(constants);
        }

        // Moves the robot based on user input. Updates target
        // vectors and calls the motor motion commands
        //
        // Params:
        //    componentX      - the X component of the motor's movement vector
        //    componentY      - the X component of the motor's movement vector
        //
        //    thetaOffset_rad - an angle to rotate the input vector by. This is useful
        //                      because the of the discrepancy in where the 0 mark is for
        //                      the controller vs the 0 mark for input from a joystick
        void fixedMoveRobotCentric(double inputX, double inputY, double w, cartesian_vector target);

        // Moves the robot based on user input. Updates target
        // vectors and calls the motor motion commands
        //
        // Params:
        //    inputX          - the target x direction, normalised to [-1, 1]
        //    inputY          - the target y direction, normalised to [-1, 1]
        //    w               - the target radial velocity, normalised to [-1, 1] where 1
        //                      is move full clockwise
        //    thetaOffset_rad - an angle to rotate the input vector by. This is useful
        //                      because the of the discrepancy in where the 0 mark is for
        //                      the controller vs the 0 mark for input from a joystick
        void moveRobotCentric(double inputX, double inputY, double w, double thetaOffset_rad);


        void moveSingular(double inputX, double inputY, double w);
};


#endif
