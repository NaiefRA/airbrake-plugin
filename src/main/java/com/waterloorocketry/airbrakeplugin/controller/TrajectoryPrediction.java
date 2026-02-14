package com.waterloorocketry.airbrakeplugin.controller;

public class TrajectoryPrediction {

    // --- Physics Constants ---
    private static final double GRAV_AT_SEA_LVL = 9.80665; // m/s^2
    private static final double EARTH_MEAN_RADIUS = 6371009.0; // m
    private static final double TIME_STEP = 0.05; // s
    private static final double ROCKET_BURNOUT_MASS_KG = 42.5288206112; // 93.76 lbs
    private static final double LAUNCH_PAD_ELEVATION_M = 295.0; // m

    // NOTE: EXTENSION_REFERENCE was in your .h file. Assuming 0.0 for coast
    // prediction.
    private static final double EXTENSION_REFERENCE = 0.58;

    // Cubic drag force polynomial coeffs for extensions 0-100% in 10% intervals
    // Format: {p00, p10, p01, p20, p11, p02, p30, p21, p12, p03}
    private static final double[][] DRAG_POLYNOMIAL_COEFFS = {
            { 232.2951, 244.7010, -75.1435, 64.3402, -79.5220, 11.7309, -0.8306, -20.4344, 9.7041, -0.6148 }, // 0% ext
            { 235.8993, 249.2100, -76.2767, 65.8251, -81.2931, 12.0289, -0.8408, -21.0236, 9.9787, -0.7853 }, // 10% ext
            { 245.6886, 260.2967, -80.2111, 69.3746, -85.4361, 12.5705, -0.6199, -22.1676, 10.4297, -0.6666 }, // 20%
            { 253.9691, 270.2409, -83.5032, 72.3919, -89.3117, 13.3189, -0.7371, -23.2489, 11.0822, -0.7471 }, // 30%
            { 263.5127, 280.7695, -86.9338, 75.8929, -93.6884, 14.1591, -0.3771, -24.5324, 11.7445, -0.9399 }, // 40%
            { 272.4592, 290.5670, -90.1040, 78.8810, -97.0343, 14.5126, -0.1988, -25.6374, 12.0348, -0.7327 }, // 50%
            { 284.8368, 304.7727, -94.4923, 82.4469, -101.4462, 15.1080, -0.6357, -26.5433, 12.4927, -0.8323 }, // 60%
            { 296.2638, 317.3919, -98.5746, 86.2809, -106.2663, 15.9556, -0.5224, -28.0106, 13.2557, -0.8541 }, // 70%
            { 303.1856, 325.1022, -100.9674, 88.7552, -109.3743, 16.5627, -0.4253, -28.9892, 13.8034, -0.9447 }, // 80%
            { 316.4963, 339.6502, -104.5570, 92.4088, -114.1954, 16.9114, -0.5681, -30.4995, 13.9269, -1.2933 }, // 90%
            { 340.9146, 367.1520, -114.8622, 101.0439, -123.1214, 18.4047, -0.1879, -31.8071, 15.4422, -1.1456 } // 100%
                                                                                                                 // ext
    };

    // --- Inner Classes for State Tracking ---
    private static class RK4State {
        double vy_m_s;
        double vx_m_s;
        double y_m;
    }

    private static class Accelerations {
        double ay_m_s2;
        double ax_m_s2;
    }

    // --- Math Methods ---

    private static double evaluateCubic2Variable(double[] poly, double x, double y) {
        return poly[0] + poly[1] * x + poly[2] * y + poly[3] * x * x +
                poly[4] * x * y + poly[5] * y * y + poly[6] * x * x * x +
                poly[7] * x * x * y + poly[8] * x * y * y + poly[9] * y * y * y;
    }

    private static double dragAccel_m_s2(double extension, double speed_m_s, double altitude_m) {
        if (extension < 0.0)
            extension = 0.0;
        if (extension > 1.0)
            extension = 1.0;

        double[] poly = new double[10];
        double extensionX10 = extension * 10.0;
        int index = (int) extensionX10;

        if (extensionX10 == index || index == 10) {
            System.arraycopy(DRAG_POLYNOMIAL_COEFFS[index], 0, poly, 0, 10);
        } else {
            // Linear interpolation between polynomial sets
            double[] p1 = DRAG_POLYNOMIAL_COEFFS[index];
            double[] p2 = DRAG_POLYNOMIAL_COEFFS[index + 1];
            double diff = extensionX10 - index;
            for (int i = 0; i < 10; i++) {
                poly[i] = diff * p2[i] + (1.0 - diff) * p1[i];
            }
        }

        // Divide drag force by mass to get acceleration
        for (int i = 0; i < 10; i++) {
            poly[i] /= ROCKET_BURNOUT_MASS_KG;
        }

        if (speed_m_s < 34.0) {
            return 0.0;
        }

        double x = (speed_m_s - 273.9) / 148.7;
        double y = (altitude_m + LAUNCH_PAD_ELEVATION_M - 5000.0) / 3172.0;
        double drag = evaluateCubic2Variable(poly, x, y);

        return Math.max(drag, 0.0);
    }

    private static double gravitationalAccel_m_s2(double altitude_m) {
        return GRAV_AT_SEA_LVL * Math.pow(EARTH_MEAN_RADIUS / (EARTH_MEAN_RADIUS + altitude_m), 2);
    }

    private static Accelerations getAccels(double extension, double vx_m_s, double vy_m_s, double y_m) {
        double speed_m_s = Math.sqrt(vy_m_s * vy_m_s + vx_m_s * vx_m_s);
        double ad_m_s2 = -dragAccel_m_s2(extension, speed_m_s, y_m);
        double ag_m_s2 = -gravitationalAccel_m_s2(y_m);

        Accelerations accel = new Accelerations();
        accel.ay_m_s2 = (speed_m_s > 0) ? (ad_m_s2 * vy_m_s / speed_m_s + ag_m_s2) : ag_m_s2;
        accel.ax_m_s2 = (speed_m_s > 0) ? (ad_m_s2 * vx_m_s / speed_m_s) : 0;
        return accel;
    }

    private static RK4State rk4(double h_s, double extension, RK4State state) {
        Accelerations accels = getAccels(extension, state.vx_m_s, state.vy_m_s, state.y_m);
        double ka1 = h_s * state.vy_m_s;
        double kvY1 = h_s * accels.ay_m_s2;
        double kvX1 = h_s * accels.ax_m_s2;

        accels = getAccels(extension, state.vx_m_s + kvX1 / 2, state.vy_m_s + kvY1 / 2, state.y_m + ka1 / 2);
        double ka2 = h_s * (state.vy_m_s + h_s * kvY1 / 2);
        double kvY2 = h_s * accels.ay_m_s2;
        double kvX2 = h_s * accels.ax_m_s2;

        accels = getAccels(extension, state.vx_m_s + kvX2 / 2, state.vy_m_s + kvY2 / 2, state.y_m + ka2 / 2);
        double ka3 = h_s * (state.vy_m_s + h_s * kvY2 / 2);
        double kvY3 = h_s * accels.ay_m_s2;
        double kvX3 = h_s * accels.ax_m_s2;

        accels = getAccels(extension, state.vx_m_s + kvX3, state.vy_m_s + kvY3, state.y_m + ka3);
        double ka4 = h_s * (state.vy_m_s + h_s * kvY3);
        double kvY4 = h_s * accels.ay_m_s2;
        double kvX4 = h_s * accels.ax_m_s2;

        RK4State updatedState = new RK4State();
        updatedState.y_m = state.y_m + (ka1 + 2 * ka2 + 2 * ka3 + ka4) / 6.0;
        updatedState.vy_m_s = state.vy_m_s + (kvY1 + 2 * kvY2 + 2 * kvY3 + kvY4) / 6.0;
        updatedState.vx_m_s = state.vx_m_s + (kvX1 + 2 * kvX2 + 2 * kvX3 + kvX4) / 6.0;

        return updatedState;
    }

    /**
     * Replaces the old JNI call.
     * 
     * @param rocketState Current rocket state from OR
     * @return max predicted apogee
     */
    public static double get_max_altitude(Controller.RocketState rocketState) {
        double vX = Math
                .sqrt(rocketState.velocityX * rocketState.velocityX + rocketState.velocityY * rocketState.velocityY);
        double vY = rocketState.velocityZ; // In OR, +Z is Up (Y in the C++ math)
        double y_m = rocketState.positionZ;

        double prevAlt = 0.0;
        RK4State state = new RK4State();
        state.y_m = y_m;
        state.vy_m_s = vY;
        state.vx_m_s = vX;

        // Propagate RK4 forward until vertical velocity is zero / altitude stops
        // increasing
        while (state.y_m >= prevAlt) {
            prevAlt = state.y_m;
            state = rk4(TIME_STEP, EXTENSION_REFERENCE, state);
        }

        return prevAlt;
    }
}