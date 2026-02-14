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

//         // TODO  CALC CD from extension

//         // 3. Calculate Drag Force
//         // Formula: 1/2 * density * velocity^2 * Cd * Area
//         return 0.5 * density * velocity * velocity * CD * MAX_AREA;
//     }
// }

package com.waterloorocketry.airbrakeplugin.airbrake;

import com.waterloorocketry.airbrakeplugin.simulated.AirDensity;

/**
 * Airbrakes using flat plate theory
 */
public class SimulatedAirbrakes implements Airbrakes {

    private final double cd;

    public SimulatedAirbrakes(double cd) {
        this.cd = cd;
    }

    @Override
    public double calculateDragForce(double extension, double velocity, double altitude) {

        double airbrakesArea = 0.007322 * 4;
        double maxAngle = Math.toRadians(60);

        double theta = extension * maxAngle;
        double rho = AirDensity.getAirDensityAtAltitude(altitude);
        double CD = 2.5 * Math.sin(theta) * Math.sin(theta) + 0.074 * Math.cos(theta);

        return 0.5 * rho * velocity * velocity * (airbrakesArea) * CD;
    }
}
