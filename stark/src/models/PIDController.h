#pragma once

namespace stark::models
{
    /**
     * @brief A PID controller implementation, for example, for motor control.
     *   A Proportional-Integral-Derivative (PID) controller is a feedback control 
     *   loop mechanism widely used in control systems to achieve desired outcomes. 
     *   It aims to minimize the error between a desired setpoint and the actual process 
     *   variable. The controller adjusts its control effort (in this case, torque 
     *   applied to a motor) based on three components:
     *
     *   Proportional (P) term: Responds to the current error. A higher error results in a stronger response.
     *   Integral (I) term: Responds to the accumulated sum of past errors. It helps eliminate steady-state errors.
     *   Derivative (D) term: Responds to the rate of change of the error. It helps dampen rapid changes and reduces overshooting.     
     * 
     * https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
     */
    class PIDController 
    {
    public:
        /**
        * @brief Constructor to initialize PID controller gains.
        * @param kp Proportional gain.
        * @param ki Integral gain.
        * @param kd Derivative gain.
        */
        PIDController(double kp, double ki, double kd)
            : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}
        PIDController() {};
        double operator()(double target, double current, double dt) 
        {
            double error = target - current;
            integral += error * dt;
            double derivative = (error - prevError) / dt;

            double output = kp * error + ki * integral + kd * derivative;

            prevError = error;
            return output;
        }

    private:
        double kp = 0.5;
        double ki = 0.5;
        double kd = 1.0;
        double prevError = 0.0;
        double integral = 0.0;
    };
}