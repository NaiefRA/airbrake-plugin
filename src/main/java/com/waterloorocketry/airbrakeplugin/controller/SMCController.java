package com.waterloorocketry.airbrakeplugin.controller;

public class SMCController implements Controller {
    private final float targetAltitude;
    private final float c;
    private final float K_smc;
    private final float Phi;

    private double lastError = 0;
    private double lastTime = -1;

    public SMCController(float target, float c, float K, float phi) {
        this.targetAltitude = target;
        this.c = c;
        this.K_smc = K;
        this.Phi = phi;
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

        double deltaH = TrajectoryPrediction.get_apogee_delta(h, vMag, 30.0, inclDeg);

        double predictedApogee = h + deltaH;
        double error = predictedApogee - targetAltitude;

        System.out.printf(
                "Predicted Apogee: %6.2f \n", predictedApogee);

        if (lastTime == -1) {
            lastTime = currentTime;
            lastError = error;
            return 0.0;
        }

        double dt = currentTime - lastTime;
        if (dt <= 0) {
            return currentExtension;
        }

        double error_dot = (error - lastError) / dt;
        double S = c * error + error_dot;

        double output = K_smc * Math.tanh(S / Phi);

        lastError = error;
        lastTime = currentTime;

        double extensionError = output - currentExtension;
        if (extensionError > rateLimit) {
            output = currentExtension + rateLimit;
        } else if (extensionError < -rateLimit) {
            output = currentExtension - rateLimit;
        }

        return Math.max(0.0, Math.min(1.0, output));
    }
}