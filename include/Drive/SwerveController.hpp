// SwerveController.hpp
// Contains code for controlling a standard swerve bot. Configuration
// for all the motors and swerve modules occurs in the constructor
//
// Author: Aiden Carney

#ifndef SWERVECONTROLLER_HPP
#define SWERVECONTROLLER_HPP

#include "SwerveModule.hpp"
#include "CAN/SparkMaxMC.hpp"


class SwerveController {
    public:
        enum drive_mode {
            robot_centric,
            field_centric
        };

        // ctor - constructs the swerve drive object based on the motors.
        // Performs configuration steps
        //
        // Params:
        //    xxDriveMotor - reference to the given drive motor (big NEO)
        //    xxPivotMotor - reference to the given pivot motor (little NEO)
        SwerveController(
            SparkMaxMC& neDriveMotor,
            SparkMaxMC& nePivotMotor,
            SparkMaxMC& nwDriveMotor,
            SparkMaxMC& nwPivotMotor,
            SparkMaxMC& seDriveMotor,
            SparkMaxMC& sePivotMotor,
            SparkMaxMC& swDriveMotor,
            SparkMaxMC& swPivotMotor,
            bool useAltEncoder=true
        );


        // dtor - frees memory allocated for swerve modules
        //
        // Params:
        //    None
        // Return:
        //    None
        ~SwerveController();


        // Calibrates the swerve modules to a known offset based on
        // a given configuration file
        //
        // Params:
        //    calibrationConfigFile - the file to read the calibration from
        // Return:
        //    int - 0 on success, -1 on failure to read
        int importCalibration(const char calibrationConfigFile[255]);


        // Sets the current positions of the robot as the calibration configuration
        // to be saved to a new file. Uses an average of the sensor values over a
        // period of time to ensure best results
        //
        // Params:
        //    calibrationConfigFile - the file to read the calibration from
        //    calibrationTime_ms    - how long to average sensor values for in ms
        // Return:
        //    int - 0 on success, -1 on failure to write
        int calibrate(const char calibrationConfigFile[255], int calibrationTime);


        // Sets the sensitivity of the input. This is a simple constant multiplied
        // to the output of the velocity controller for the drive motors
        //
        // Params:
        //    s - the new sensitivity
        // Return:
        //    None
        void setSensitivity(int s);


        // sets the new drive mode for the swerve drivetrain
        //
        // Params:
        //    newMode - the new drive mode
        // Return:
        //    None
        void setDriveMode(drive_mode newMode) {mode = newMode;}


        // Moves the drivetrain based on the current mode and the
        // given inputs
        //
        // Params:
        //    inputX - the normalised x input
        //    inputY - the normalised y input
        //    w      - the normalised omega input
        void fixedMove(double inputX, double inputY, double w);

        // Moves the drivetrain based on the current mode and the
        // given inputs
        //
        // Params:
        //    inputX - the normalised x input
        //    inputY - the normalised y input
        //    w      - the normalised omega input
        void move(double inputX, double inputY, double w);

    private:
        SparkMaxMC* neDrive;
        SparkMaxMC* nePivot;
        SparkMaxMC* nwDrive;
        SparkMaxMC* nwPivot;
        SparkMaxMC* seDrive;
        SparkMaxMC* sePivot;
        SparkMaxMC* swDrive;
        SparkMaxMC* swPivot;

        SwerveModule* ne; 
        SwerveModule* nw;
        SwerveModule* se;
        SwerveModule* sw;

        double sensitivity = 1.0;
        drive_mode mode = robot_centric;
};


#endif
