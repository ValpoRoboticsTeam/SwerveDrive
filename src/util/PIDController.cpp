#include <cmath>

#include "util/PIDController.hpp"


// initializes clock
PIDController::PIDController() {
    prevTime = std::chrono::steady_clock::now();
}


// sets values of constants and initializes clock
PIDController::PIDController(pidf_constants newConstants) {
    constants = newConstants;
    prevTime = std::chrono::steady_clock::now();
}


// resets values for a new setpoint
void PIDController::newSetpoint(double sp) {
    setpoint = sp;
    integral = 0;
    prevError = 0;
    prevTime = std::chrono::steady_clock::now();
}


// calculates pid output. Factors in the change in time so technically this
// loop does not need to be called with a constant period
double PIDController::step(double feedback) {
    double error = setpoint - feedback;
    if(wrapAngle && error > M_PI) {
        error -= 2 * M_PI;
    } else if(wrapAngle && error < -M_PI) {
        error += 2 * M_PI;
    }

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds> (now - prevTime).count();
    prevTime = now;

    double p = error * constants.kP;

    if(abs(error) <= constants.kIZone || constants.kIZone == 0.0) {
        integral = integral + (error * dt);
    } else {
        integral = 0;
    }
    double i = integral * constants.kI;

    double d = ((error - prevError) / dt) * constants.kD;
    prevError = error;

    double f = setpoint * constants.kF;

    double output = p + i + d + f;
    output = fmax(output, constants.kOutputMin);  // don't allow to be less than kOutputMin
    output = fmin(output, constants.kOutputMax);  // don't allow to be greater than kOutputMax

    return output;
}
