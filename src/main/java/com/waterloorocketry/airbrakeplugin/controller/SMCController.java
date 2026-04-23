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

        // Predict where the rocket will coast to
        double predictedApogee = TrajectoryPrediction.get_max_altitude(rocketState);

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
        double error_dot = (error - lastError) / dt;

        // SMC Logic

        double S = c * error + error_dot;

        double output = K_smc * Math.tanh(S / Phi);

        // update for next loop
        lastError = error;
        lastTime = currentTime;

        // Clamp output to physical limits
        if (output > 1.0)
            output = 1.0;
        if (output < 0.0)
            output = 0.0;

        // -------------------------
        // STABILITY FILTERS
        // -------------------------

        double extensionError = output - currentExtension;

        if (extensionError > rateLimit)
            return currentExtension + rateLimit;

        if (extensionError < -rateLimit)
            return currentExtension - rateLimit;

        return output;
    }
}