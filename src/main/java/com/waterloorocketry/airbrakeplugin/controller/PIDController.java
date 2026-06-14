package com.waterloorocketry.airbrakeplugin.controller;

public class PIDController implements Controller {

    private final float targetAltitude;
    private final float Kp;
    private final float Ki;
    private final float Kd;
    private final float iLimit;

    private double lastError = 0;
    private double integralSum = 0;
    private double lastTime = -1;

    public PIDController(float targetAltitude, float Kp, float Ki, float Kd, float iLimit) {
        this.targetAltitude = targetAltitude;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.iLimit = iLimit;
    }

    @Override
    public double calculateTargetExt(RocketState rocketState, double currentTime, double currentExtension,
            double rateLimit) {

        double h = rocketState.positionZ;
        double vMag = Math.sqrt(
                rocketState.velocityX * rocketState.velocityX +
                        rocketState.velocityY * rocketState.velocityY +
                        rocketState.velocityZ * rocketState.velocityZ);

        double inclDeg = 0.0;
        if (vMag > 0.0) {
            inclDeg = Math.toDegrees(Math.acos(rocketState.velocityZ / vMag));
        }

        // 30.0 degrees corresponds to the 0.5 extension baseline
        double deltaH = TrajectoryPrediction.get_apogee_delta(h, vMag, 30.0, inclDeg);
        double predictedApogee = h + deltaH;
        if (lastTime == -1) {
            lastTime = currentTime;
            return 0.0;
        }

        double dt = currentTime - lastTime;
        if (dt <= 0)
            return currentExtension;

        double error = predictedApogee - targetAltitude;

        double pTerm = Kp * error;

        integralSum += error * dt;
        if (integralSum > iLimit)
            integralSum = iLimit;
        if (integralSum < -iLimit)
            integralSum = -iLimit;
        double iTerm = Ki * integralSum;

        double derivative = (error - lastError) / dt;
        double dTerm = Kd * derivative;

        double output = pTerm + iTerm + dTerm;

        lastError = error;
        lastTime = currentTime;

        if (output > 1.0)
            output = 1.0;
        if (output < 0.0)
            output = 0.0;

        double extensionError = output - currentExtension;

        if (extensionError > rateLimit)
            return currentExtension + rateLimit;

        if (extensionError < -rateLimit)
            return currentExtension - rateLimit;

        return output;
    }
}