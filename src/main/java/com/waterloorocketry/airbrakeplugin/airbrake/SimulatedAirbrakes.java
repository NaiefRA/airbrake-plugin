// package com.waterloorocketry.airbrakeplugin.airbrake;

// import com.waterloorocketry.airbrakeplugin.simulated.SimulatedDragForceInterpolator;

// /**
//  * Airbrakes using CFD simulated values
//  */
// public class SimulatedAirbrakes implements Airbrakes {
//     private final SimulatedDragForceInterpolator interp = new SimulatedDragForceInterpolator();

//     @Override
//     public double calculateDragForce(double extension, double velocity, double altitude) {
//         return interp.compute(new SimulatedDragForceInterpolator.Data(extension, velocity, altitude));
//     }
// }

// package com.waterloorocketry.airbrakeplugin.airbrake; 

// import com.waterloorocketry.airbrakeplugin.simulated.AirDensity;

// /**
//  * Airbrakes using standard drag equation: F = 0.5 * rho * v^2 * Cd * A
//  */
// public class SimulatedAirbrakes implements Airbrakes {

//     // HARDCODED CONSTANTS
//     private static final double CD = 1.5;        // Drag Coefficient
//     private static final double MAX_AREA = 4 * 0.007322; // Maximum area of all 4 airbrakes in m^2

//     @Override
//     public double calculateDragForce(double extension, double velocity, double altitude) {
//         // 1. Get Air Density at current altitude (using existing project utility)
//         double density = AirDensity.getAirDensityAtAltitude(altitude);

//         // 3. Calculate Drag Force
//         // Formula: 1/2 * density * velocity^2 * Cd * Area
//         return 0.5 * density * velocity * velocity * CD * MAX_AREA;
//     }
// }

package com.waterloorocketry.airbrakeplugin.airbrake;

// import com.waterloorocketry.airbrakeplugin.simulated.AirDensity;

/**
 * Airbrakes using flat plate theory
 */
public class SimulatedAirbrakes implements Airbrakes {

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

    private double getTotalRocketCD(double targetMach, double targetDeflection) {

        double mach = Math.max(0.40, Math.min(targetMach, 0.85));
        double delta = Math.max(0.0, Math.min(targetDeflection, 60.0));

        int i = 0;
        while (i < CFD_CD_MATRIX.length - 2 && CFD_CD_MATRIX[i + 1][0] <= mach) {
            i++;
        }

        double mBelow = CFD_CD_MATRIX[i][0];
        double mAbove = CFD_CD_MATRIX[i + 1][0];
        double t = (mach - mBelow) / (mAbove - mBelow);

        // Linear interpolation across mach
        double cd0 = CFD_CD_MATRIX[i][1] + t * (CFD_CD_MATRIX[i + 1][1] - CFD_CD_MATRIX[i][1]);
        double cd15 = CFD_CD_MATRIX[i][2] + t * (CFD_CD_MATRIX[i + 1][2] - CFD_CD_MATRIX[i][2]);
        double cd30 = CFD_CD_MATRIX[i][3] + t * (CFD_CD_MATRIX[i + 1][3] - CFD_CD_MATRIX[i][3]);
        double cd45 = CFD_CD_MATRIX[i][4] + t * (CFD_CD_MATRIX[i + 1][4] - CFD_CD_MATRIX[i][4]);
        double cd60 = CFD_CD_MATRIX[i][5] + t * (CFD_CD_MATRIX[i + 1][5] - CFD_CD_MATRIX[i][5]);

        // Quadratic interpolation across deflection
        double totalCD;
        if (delta <= 22.5) {
            totalCD = cd0 * ((delta - 15) * (delta - 30)) / 450.0
                    + cd15 * ((delta) * (delta - 30)) / -225.0
                    + cd30 * ((delta) * (delta - 15)) / 450.0;
        } else if (delta <= 37.5) {
            totalCD = cd15 * ((delta - 30) * (delta - 45)) / 450.0
                    + cd30 * ((delta - 15) * (delta - 45)) / -225.0
                    + cd45 * ((delta - 15) * (delta - 30)) / 450.0;
        } else {
            totalCD = cd30 * ((delta - 45) * (delta - 60)) / 450.0
                    + cd45 * ((delta - 30) * (delta - 60)) / -225.0
                    + cd60 * ((delta - 30) * (delta - 45)) / 450.0;
        }

        return totalCD;
    }

    // private static final double REF_AREA = 0.02; // m^2
    private static final double MAX_ANGLE_RAD = Math.toRadians(60.0);

    public SimulatedAirbrakes() {
    }

    @Override
    public double calculateDragCoefficient(double extension, double velocity, double altitude) {

        double theta = extension * MAX_ANGLE_RAD;
        // double rho = AirDensity.getAirDensityAtAltitude(altitude);
        double mach = velocity / (Math.sqrt(1.4 * 287.05 * (288.15 - (0.0065 * altitude))));

        double deflectionDeg = Math.toDegrees(theta);
        double totalCD = getTotalRocketCD(mach, deflectionDeg);

        // return 0.5 * rho * velocity * velocity * REF_AREA * totalCD;
        return totalCD;

    }
}
