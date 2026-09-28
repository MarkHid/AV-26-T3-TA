#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"

class Controller : public IController {
public:
    double update(double target, double measured, double dt) override {
        const double kp = 1.5;
        const double ki = 0.01;
        const double kd = 0.02;
        const double maxCommand = 15.0;
        const double integralLimit = 8.0;

        double error = target - measured;

        // Clear stored error once the target is reached or crossed. This
        // stops the integral term from continuing to push past the target.
        if ((error > -0.05 && error < 0.05) ||
            (!firstUpdate && error * previousError < 0.0)) {
            integral = 0.0;
        }

        integral += error * dt;
        if (integral > integralLimit)
            integral = integralLimit;
        else if (integral < -integralLimit)
            integral = -integralLimit;

        double derivative = 0.0;
        if (!firstUpdate && dt > 0.0)
            derivative = (error - previousError) / dt;

        double command = kp * error + ki * integral + kd * derivative;

        if (command > maxCommand)
            command = maxCommand;
        else if (command < -maxCommand)
            command = -maxCommand;

        previousError = error;
        firstUpdate = false;
        return command;
    }

    void reset() override {
        integral = 0.0;
        previousError = 0.0;
        firstUpdate = true;
    }

private:
    double integral = 0.0;
    double previousError = 0.0;
    bool firstUpdate = true;
};
