#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    double angle = 0.0;
    double motorAngle = 0.0;
    double velocity = 0.0;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {
        const double gain = 1.3;
        const double timeConstant = 0.1;
        const double backlash = 2.6;

        // The actuator takes some time to reach the commanded velocity.
        double targetVelocity = gain * u_cmd;
        velocity += (targetVelocity - velocity) * dt / timeConstant;

        // Work out the position of the motor before the backlash is applied.
        motorAngle += velocity * dt;

        // The output only moves after the motor has crossed the backlash gap.
        if (motorAngle > angle + backlash) {
            angle = motorAngle - backlash;
        } else if (motorAngle < angle - backlash) {
            angle = motorAngle + backlash;
        }

        return std::round(angle / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() {
        angle = 0.0;
        motorAngle = 0.0;
        velocity = 0.0;
    }
};
