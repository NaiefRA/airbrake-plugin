package com.waterloorocketry.airbrakeplugin.controller;

import com.waterloorocketry.airbrakeplugin.jni.ProcessorCalculations;

public class PIDController implements Controller {

    // -------------------------
    // Fields
    // -------------------------

    private final float targetAltitude;
    private final float Kp;
    private final float Ki;
    private final float Kd;
    private final float iLimit;

    private double lastError = 0;
    private double integralSum = 0;
    private double lastTime = -1;

    // -------------------------
    // Constructor
    // -------------------------
    public PIDController(float targetAltitude, float Kp, float Ki, float Kd, float iLimit) {
        this.targetAltitude = targetAltitude;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.iLimit = iLimit;
    }

    // -------------------------
    // PID Controller Method
    // -------------------------
    @Override
    public double calculateTargetExt(RocketState rocketState, double currentTime, double currentExtension) {

        // Compute lateral velocity magnitude
        double vX = Math.sqrt(
            rocketState.velocityX * rocketState.velocityX +
            rocketState.velocityY * rocketState.velocityY
        );

        // Predict where the rocket will coast to
        float predictedApogee = ProcessorCalculations.getMaxAltitude(
            (float) rocketState.velocityZ,
            (float) vX,
            (float) rocketState.positionZ
        );

        // First loop iteration
        if (lastTime == -1) {
            lastTime = currentTime;
            return 0.0;
        }

        // Compute dt
        double dt = currentTime - lastTime;
        if (dt <= 0)
            return currentExtension;

        // Error term (positive means overshooting target)
        double error = predictedApogee - targetAltitude;

        // PID Components
        double pTerm = Kp * error;

        // Integral term with anti-windup
        integralSum += error * dt;
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        double iTerm = Ki * integralSum;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = Kd * derivative;

        // PID Output (0–1 airbrake extension)
        double output = pTerm + iTerm + dTerm;

        lastError = error;
        lastTime = currentTime;

        // Clamp output to physical limits
        if (output > 1.0) output = 1.0;
        if (output < 0.0) output = 0.0;

        // -------------------------
        //  STABILITY FILTERS
        // -------------------------

        double extensionError = output - currentExtension;

  
        // double th = 2;  // 2%
        // if (Math.abs(extensionError) < th) {
        //     return currentExtension;
        // }

       
        double maxRatePerSecond = 0.20;  // 20% per second
        double maxStep = maxRatePerSecond * dt;

        if (extensionError > 0.04)
            return currentExtension + 0.04;

        if (extensionError < -0.04)
            return currentExtension - 0.04;


        return output;
    }
}

