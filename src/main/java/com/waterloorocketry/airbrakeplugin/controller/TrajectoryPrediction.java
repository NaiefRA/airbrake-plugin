package com.waterloorocketry.airbrakeplugin.controller;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class TrajectoryPrediction {

        // Define the exact dimensions used in your Python generation script
        private static final int ALT_SIZE = 43;
        private static final int VEL_SIZE = 41;
        private static final int DEF_SIZE = 21;
        private static final int INC_SIZE = 11;

        private static final double[][][][] APOGEE_DELTA = new double[ALT_SIZE][VEL_SIZE][DEF_SIZE][INC_SIZE];

        static {
                try (InputStream is = TrajectoryPrediction.class.getResourceAsStream("/apogee_table.bin")) {
                        if (is == null) {
                                throw new IOException("Could not find apogee_table.bin in resources path.");
                        }

                        byte[] bytes = is.readAllBytes();
                        ByteBuffer buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);

                        for (int x = 0; x < ALT_SIZE; x++) {
                                for (int y = 0; y < VEL_SIZE; y++) {
                                        for (int z = 0; z < DEF_SIZE; z++) {
                                                for (int w = 0; w < INC_SIZE; w++) {
                                                        APOGEE_DELTA[x][y][z][w] = buffer.getDouble();
                                                }
                                        }
                                }
                        }
                } catch (IOException e) {
                        System.err.println("Error initializing apogee lookup table: " + e.getMessage());
                        e.printStackTrace();
                }
        }

        public static double get_apogee_delta(double alt, double vel, double defDeg, double inclDeg) {
                // 1. Adjust clamps to your actual table bounds (Max alt: 3100, Max incl: 10.0)
                alt = Math.max(1000.0, Math.min(alt, 3100.0));
                vel = Math.max(0.0, Math.min(vel, 800.0)); // Note: 41 Machs * 340m/s -> roughly 800m/s max
                defDeg = Math.max(0.0, Math.min(defDeg, 60.0));
                inclDeg = Math.max(0.0, Math.min(inclDeg, 10.0));

                double x = (alt - 1000.0) / 50.0;
                double y = vel / 6.8; // Assuming 0.02 Mach steps ~ 6.8 m/s. Adjust to your actual velocity step in
                                      // m/s.
                double z = defDeg / 3.0;
                double w = inclDeg / 1.0;

                // 2. Clamp the base index calculations to prevent floating-point overflow
                int x0 = Math.min((int) Math.floor(x), ALT_SIZE - 1);
                int y0 = Math.min((int) Math.floor(y), VEL_SIZE - 1);
                int z0 = Math.min((int) Math.floor(z), DEF_SIZE - 1);
                int w0 = Math.min((int) Math.floor(w), INC_SIZE - 1);

                int x1 = Math.min(x0 + 1, ALT_SIZE - 1);
                int y1 = Math.min(y0 + 1, VEL_SIZE - 1);
                int z1 = Math.min(z0 + 1, DEF_SIZE - 1);
                int w1 = Math.min(w0 + 1, INC_SIZE - 1);

                double xd = x - x0;
                double yd = y - y0;
                double zd = z - z0;
                double wd = w - w0;

                double c0_w0 = interp3D(x0, x1, y0, y1, z0, z1, xd, yd, zd, w0);
                double c1_w1 = interp3D(x0, x1, y0, y1, z0, z1, xd, yd, zd, w1);

                return c0_w0 * (1 - wd) + c1_w1 * wd;
        }

        private static double interp3D(int x0, int x1, int y0, int y1, int z0, int z1,
                        double xd, double yd, double zd, int w) {
                double c00 = APOGEE_DELTA[x0][y0][z0][w] * (1 - xd) + APOGEE_DELTA[x1][y0][z0][w] * xd;
                double c01 = APOGEE_DELTA[x0][y0][z1][w] * (1 - xd) + APOGEE_DELTA[x1][y0][z1][w] * xd;
                double c10 = APOGEE_DELTA[x0][y1][z0][w] * (1 - xd) + APOGEE_DELTA[x1][y1][z0][w] * xd;
                double c11 = APOGEE_DELTA[x0][y1][z1][w] * (1 - xd) + APOGEE_DELTA[x1][y1][z1][w] * xd;

                double c0 = c00 * (1 - yd) + c10 * yd;
                double c1 = c01 * (1 - yd) + c11 * yd;

                return c0 * (1 - zd) + c1 * zd;
        }
}