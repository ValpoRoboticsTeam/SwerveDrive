// PIDController.hpp
// Contains generic PID controller implementation
//
// Author: Aiden Carney

#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <chrono>


// data for the pid constants used by the generic controller
// as well as the sparkmax motor controller.
typedef struct {
    float kP;
    float kI;
    float kD;
    float kF;
    float kIZone;
    float kDFilter;    // not used, but don't touch
    float kOutputMin;
    float kOutputMax;
} pidf_constants;


class PIDController {

    private:
        pidf_constants constants = {0, 0, 0, 0, 0, 0, -1, 1};
        bool wrapAngle = false;

        double setpoint = 0;
        double integral = 0;
        double prevError = 0;
        std::chrono::steady_clock::time_point prevTime;


    public:
        // ctor - sets default values for constants and initializes
        // the other member variables
        //
        // Params:
        //    None
        // Return:
        //    the new instance
        PIDController();


        // ctor - sets values of constants and initializes the other
        // member variables
        //
        // Params:
        //    newConstants - the constants to use
        // Return:
        //    the new instance
        PIDController(pidf_constants newConstants);


        // Sets new contants for the controller
        //
        // Params:
        //    newConstants - the struct of new constants for the controller
        // Return:
        //    None
        void setConstants(pidf_constants& newConstants) {constants = newConstants;}


        // sets whether angle wrapping for PID controller is enabled. 
        // If enabled, any error outside of [-pi, pi] will be scaled
        // back to this range. NOTE: wrapping is based on error in
        // units of radians, not degrees
        void setWrapAngle(bool wrap) {wrapAngle = wrap;}


        // Sets a new setpoint for the controller
        //
        // Params:
        //    sp - the new setpoint
        // Return:
        //    None
        void newSetpoint(double sp);


        // returns the current setpoint
        //
        // Params:
        //    None
        // Return:
        //    double - the current setpoint
        double getSetpoint() {return setpoint;}


        // runs the PID controller and returns the output of the controller
        // between kOutputMin and kOutputMax
        double step(double feedback);

};




#endif
