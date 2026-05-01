package com.waterloorocketry.airbrakeplugin.controller;

public class TrajectoryPrediction {

    // --- Physics Constants ---
    private static final double GRAV_AT_SEA_LVL = 9.80665; // m/s^2
    private static final double EARTH_MEAN_RADIUS = 6371009.0; // m
    private static final double TIME_STEP = 0.05; // s
    private static final double ROCKET_BURNOUT_MASS_KG = 42.5288206112;
    private static final double LAUNCH_PAD_ELEVATION_M = 295.0; // m

    private static final double CFD_REF_AREA = 0.02; // m^2
    private static final double MAX_ANGLE_DEG = 60.0;
    private static final double EXTENSION_REFERENCE = 0.58;

    // [Mach, CD_0deg, CD_15deg, CD_30deg, CD_45deg, CD_60deg]
    private static final double[][] CFD_CD_MATRIX = {
            { 0.40, 0.446784, 0.657489, 1.020000, 1.399296, 1.776435 },
            { 0.45, 0.447388, 0.650148, 1.029177, 1.412706, 1.789007 },
            { 0.50, 0.448465, 0.646282, 1.034709, 1.456166, 1.805239 },
            { 0.55, 0.439217, 0.642689, 1.021036, 1.464841, 1.825660 },
            { 0.60, 0.433215, 0.640186, 1.029273, 1.477620, 1.851421 },
            { 0.65, 0.443270, 0.638743, 1.037631, 1.496000, 1.883318 },
            { 0.70, 0.434580, 0.638731, 1.050824, 1.512006, 1.923296 },
            { 0.75, 0.436587, 0.640397, 1.072332, 1.552897, 1.974154 },
            { 0.80, 0.437776, 0.643724, 1.097984, 1.612904, 2.040497 },
            { 0.85, 0.447283, 0.647051, 1.127243, 1.672369, 2.133259 }
    };

    /**
     * Calculates combined X and Y accelerations at a given state.
     * Uses a passed array to avoid object allocation in the RK4 loop.
     * out[0] = ay_m_s2, out[1] = ax_m_s2
     */
    private static void calcAccels(double extension, double vx, double vy, double y, double[] out) {
        double speed = Math.sqrt(vx * vx + vy * vy);

        // Gravity
        double ag = -GRAV_AT_SEA_LVL * Math.pow(EARTH_MEAN_RADIUS / (EARTH_MEAN_RADIUS + y), 2);

        if (speed < 0.1) {
            out[0] = ag;
            out[1] = 0.0;
            return;
        }

        // Standard Atmosphere limits (Troposphere valid up to 11km)
        double altMSL = y + LAUNCH_PAD_ELEVATION_M;
        double temperatureK = 288.15 - (0.0065 * altMSL);
        double pressurePa = 101325.0 * Math.pow(temperatureK / 288.15, 5.25588);
        double rho = pressurePa / (287.05 * temperatureK);
        double speedOfSound = Math.sqrt(1.4 * 287.05 * temperatureK);

        double mach = speed / speedOfSound;
        double deflectionDeg = extension * MAX_ANGLE_DEG;

        // Fetch CD from the CFD matrix
        double cd = getTotalRocketCD(mach, deflectionDeg);

        // F = 0.5 * rho * v^2 * Cd * A. Accel = F / m
        double dragForce = 0.5 * rho * speed * speed * cd * CFD_REF_AREA;
        double ad = -(dragForce / ROCKET_BURNOUT_MASS_KG);

        // Vectorize acceleration
        out[0] = (ad * (vy / speed)) + ag;
        out[1] = (ad * (vx / speed));
    }

    private static double getTotalRocketCD(double targetMach, double targetDeflection) {
        double mach = Math.max(0.40, Math.min(targetMach, 0.85));
        double delta = Math.max(0.0, Math.min(targetDeflection, 60.0));

        int i = 0;
        while (i < CFD_CD_MATRIX.length - 2 && CFD_CD_MATRIX[i + 1][0] <= mach) {
            i++;
        }

        double mBelow = CFD_CD_MATRIX[i][0];
        double mAbove = CFD_CD_MATRIX[i + 1][0];
        double t = (mach - mBelow) / (mAbove - mBelow);

        double cd0 = CFD_CD_MATRIX[i][1] + t * (CFD_CD_MATRIX[i + 1][1] - CFD_CD_MATRIX[i][1]);
        double cd15 = CFD_CD_MATRIX[i][2] + t * (CFD_CD_MATRIX[i + 1][2] - CFD_CD_MATRIX[i][2]);
        double cd30 = CFD_CD_MATRIX[i][3] + t * (CFD_CD_MATRIX[i + 1][3] - CFD_CD_MATRIX[i][3]);
        double cd45 = CFD_CD_MATRIX[i][4] + t * (CFD_CD_MATRIX[i + 1][4] - CFD_CD_MATRIX[i][4]);
        double cd60 = CFD_CD_MATRIX[i][5] + t * (CFD_CD_MATRIX[i + 1][5] - CFD_CD_MATRIX[i][5]);

        if (delta <= 22.5) {
            return cd0 * ((delta - 15) * (delta - 30)) / 450.0
                    + cd15 * ((delta) * (delta - 30)) / -225.0
                    + cd30 * ((delta) * (delta - 15)) / 450.0;
        } else if (delta <= 37.5) {
            return cd15 * ((delta - 30) * (delta - 45)) / 450.0
                    + cd30 * ((delta - 15) * (delta - 45)) / -225.0
                    + cd45 * ((delta - 15) * (delta - 30)) / 450.0;
        } else {
            return cd30 * ((delta - 45) * (delta - 60)) / 450.0
                    + cd45 * ((delta - 30) * (delta - 60)) / -225.0
                    + cd60 * ((delta - 30) * (delta - 45)) / 450.0;
        }
    }

    public static double get_max_altitude(Controller.RocketState rocketState) {
        double vx = Math
                .sqrt(rocketState.velocityX * rocketState.velocityX + rocketState.velocityY * rocketState.velocityY);
        double vy = rocketState.velocityZ;
        double y = rocketState.positionZ;

        double[] a = new double[2]; // Reused array for accelerations
        double h = TIME_STEP;
        // double half_h = h * 0.5;

        // Propagate RK4 forward until vertical velocity is zero (apogee)
        while (vy > 0.0) {

            // k1
            calcAccels(EXTENSION_REFERENCE, vx, vy, y, a);
            double kvY1 = h * a[0];
            double kvX1 = h * a[1];
            double ka1 = h * vy;

            // k2
            calcAccels(EXTENSION_REFERENCE, vx + kvX1 * 0.5, vy + kvY1 * 0.5, y + ka1 * 0.5, a);
            double kvY2 = h * a[0];
            double kvX2 = h * a[1];
            double ka2 = h * (vy + kvY1 * 0.5);

            // k3
            calcAccels(EXTENSION_REFERENCE, vx + kvX2 * 0.5, vy + kvY2 * 0.5, y + ka2 * 0.5, a);
            double kvY3 = h * a[0];
            double kvX3 = h * a[1];
            double ka3 = h * (vy + kvY2 * 0.5);

            // k4
            calcAccels(EXTENSION_REFERENCE, vx + kvX3, vy + kvY3, y + ka3, a);
            double kvY4 = h * a[0];
            double kvX4 = h * a[1];
            double ka4 = h * (vy + kvY3);

            // Update state variables directly
            y += (ka1 + 2 * ka2 + 2 * ka3 + ka4) / 6.0;
            vy += (kvY1 + 2 * kvY2 + 2 * kvY3 + kvY4) / 6.0;
            vx += (kvX1 + 2 * kvX2 + 2 * kvX3 + kvX4) / 6.0;

            // Break failsafe if rocket glitches and falls below launch pad
            if (y < -100)
                break;
        }

        return y;
    }
}