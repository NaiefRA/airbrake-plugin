package com.waterloorocketry.airbrakeplugin.controller;

import com.waterloorocketry.airbrakeplugin.jni.ProcessorCalculations;

public class PIDController implements Controller {

    // Tuning Parameters
    private final float targetAltitude;
    private final float Kp;
    private final float Ki;
    private final float Kd;
    private final float iLimit; // Integral Saturation Limit

    // State Variables (Memory)
    private double lastError = 0;
    private double integralSum = 0;
    private double lastTime = -1; // -1 indicates first run

    public PIDController(float targetAltitude, float Kp, float Ki, float Kd, float iLimit) {
        this.targetAltitude = targetAltitude;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.iLimit = iLimit;
    }

    @Override
    public double calculateTargetExt(RocketState rocketState, double currentTime, double currentExtension) {
        // 1. Get Prediction (Keep using C++ for this as the math is complex)
        // We calculate "vX" (Total Velocity) to pass to the predictor
        double vX = Math
                .sqrt(rocketState.velocityX * rocketState.velocityX + rocketState.velocityY * rocketState.velocityY);

        // This function predicts where we will go if we DO NOTHING (Coast)
        float predictedApogee = ProcessorCalculations.getMaxAltitude(
                (float) rocketState.velocityZ,
                (float) vX,
                (float) rocketState.positionZ);

        // 2. Calculate Time Delta (dt)
        // We need to know how much time passed since the last loop to calculate
        // Integral and Derivative
        if (lastTime == -1) {
            lastTime = currentTime;
            return 0.0; // First step, no action
        }
        double dt = currentTime - lastTime;
        if (dt <= 0)
            return currentExtension; // Prevent divide by zero if sim pauses

        // 3. Calculate Error
        // Error is positive if we are going too high (Predicted > Target)
        double error = predictedApogee - targetAltitude;

        // 4. Proportional Term
        double pTerm = Kp * error;

        // 5. Integral Term (Accumulated Error)
        integralSum += error * dt;

        // Anti-Windup (Clamping the integral so it doesn't grow infinite)
        if (integralSum > iLimit)
            integralSum = iLimit;
        if (integralSum < -iLimit)
            integralSum = -iLimit;

        double iTerm = Ki * integralSum;

        // 6. Derivative Term (Rate of change of error)
        // How fast is the error shrinking or growing?
        double derivative = (error - lastError) / dt;
        double dTerm = Kd * derivative;

        // 7. Total Output
        double output = pTerm + iTerm + dTerm;

        // 8. Update State for next loop
        lastError = error;
        lastTime = currentTime;

        // 9. Clamp Output (Airbrakes can only be 0% to 100%)
        if (output > 1.0)
            output = 1.0;
        if (output < 0.0)
            output = 0.0;

        return output;
    }
}
