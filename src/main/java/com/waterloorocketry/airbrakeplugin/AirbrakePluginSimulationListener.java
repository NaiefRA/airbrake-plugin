package com.waterloorocketry.airbrakeplugin;

import com.waterloorocketry.airbrakeplugin.airbrake.Airbrakes;
import com.waterloorocketry.airbrakeplugin.controller.Controller;
import com.waterloorocketry.airbrakeplugin.controller.TrajectoryPrediction;
import com.waterloorocketry.airbrakeplugin.simulated.Noise;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.FlightEvent;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.unit.UnitGroup;

/**
 * Connect to a simulation and listen for various events during the simulation.
 */
public class AirbrakePluginSimulationListener extends AbstractSimulationListener {
    private double lastPrintTime = 0;

    private final Airbrakes airbrakes;
    private final Controller controller;
    private final Noise noise;
    public static final FlightDataType airbrakeExtDataType = FlightDataType.getType("airbrakeExt", "airbrakeExt",
            UnitGroup.UNITS_RELATIVE);
    public static final FlightDataType predictedApogeeDataType = FlightDataType.getType("predictedApogee",
            "predictedApogee", UnitGroup.UNITS_DISTANCE);
    private double ext = 0.0;
    private final double extTime;

    public AirbrakePluginSimulationListener(Airbrakes airbrakes, Controller controller, Noise noise, double extTime) {
        super();
        this.airbrakes = airbrakes;
        this.controller = controller;
        this.noise = noise;
        this.extTime = extTime;
    }

    // Airbrakes only allowed between the given time (default 9 s) and while
    // vertical velocity > 34 m/s
    private boolean isExtensionAllowed(SimulationStatus status) {
        return status.getSimulationTime() > extTime && status.getRocketVelocity().z > 34.0;
    }

    /**
     * Runs before each timestep.
     * 
     * @param status
     * @return
     */
    @Override
    public boolean preStep(SimulationStatus status) {
        FlightDataBranch flightData = status.getFlightData();
        Controller.RocketState data = new Controller.RocketState(status);

        // Add gaussian noise to the "measured" state data if enabled
        if (noise != null) {
            java.util.Random r = new java.util.Random();
            data.velocityX = r.nextGaussian(data.velocityX, noise.getStddevVelocityX());
            data.velocityY = r.nextGaussian(data.velocityY, noise.getStddevVelocityY());
            data.velocityZ = r.nextGaussian(data.velocityZ, noise.getStddevVelocityZ());
            data.positionZ = r.nextGaussian(data.positionZ, noise.getStddevPositionZ());
        }

        // Only run controller during coast phase. If not in coast, still set ext to 0
        // (better than NaN)
        if (isExtensionAllowed(status)) {
            ext = controller.calculateTargetExt(data, status.getSimulationTime(), ext);
            if (!(0.0 <= ext && ext <= 1.0)) {
                throw new IndexOutOfBoundsException("airbrakes extension amount was not from 0 to 1");
            }
            flightData.setValue(airbrakeExtDataType, ext);
        } else {
            flightData.setValue(airbrakeExtDataType, 0);
        }

        // This is solely for graphing trajectory prediction outputs
        flightData.setValue(predictedApogeeDataType, TrajectoryPrediction.get_max_altitude(data));

        return true;
    }

    /**
     * Flight conditions for the current timestep.
     */
    private FlightConditions flightConditions = null;

    // We can't look at status.getFlightData() for anything except extension instead
    // because it would
    // apply to the last timestep
    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions)
            throws SimulationException {
        this.flightConditions = flightConditions;
        return flightConditions;
    }

    /**
     * Overrides the coefficient of drag after the aerodynamic calculations are done
     * each timestep.
     */
    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus status, AerodynamicForces forces)
            throws SimulationException {

        double extension = 0;
        double airbrakesDragForce = 0;
        double airbrakesCd = 0;
        double rocketCd = forces.getCDaxial();

        if (isExtensionAllowed(status)) {
            // Get latest flight conditions and airbrake extension
            final double velocityZ = status.getRocketVelocity().z;
            extension = status.getFlightData().getLast(airbrakeExtDataType);
            final double altitude = status.getRocketPosition().z
                    + status.getSimulationConditions().getLaunchSite().getAltitude();

            airbrakesDragForce = airbrakes.calculateDragForce(extension, velocityZ, altitude);

            // now calculating Cd from the airbrakes (using force calculated inputted (for
            // flat plate theory))
            double density = flightConditions.getAtmosphericConditions().getDensity();
            double refArea = flightConditions.getRefArea();

            double velocitySq = status.getRocketVelocity().length2();

            double dynamicPressure = 0.5 * density * velocitySq;
            double totalCd = forces.getCDaxial(); // cd from openrocket

            if (dynamicPressure > 0.0001) {
                airbrakesCd = (airbrakesDragForce / (dynamicPressure * refArea));

                totalCd = totalCd + airbrakesCd;
            }

            // OR CD + Calculated Airbrakes CD
            forces.setCDaxial(totalCd);
        }

        // printing
        if (status.getSimulationTime() - lastPrintTime > 0.2) {
            System.out.printf(
                    "Time: %6.2fs | Extended: %-5b | Ext: %3.0f%% | Drag Force: %6.2f N | Added Cd: %6.4f | Rocket Cd: %6.4f | Total Cd: %6.4f%n",
                    status.getSimulationTime(), // Time
                    extension > 0, // Whether or not airbrakes are extended (True/False)
                    extension * 100, // Extension percentage (0-100%)
                    airbrakesDragForce, // Drag Force in Newtons
                    airbrakesCd, // The Coefficient added to the rocket
                    rocketCd, // OR Cd
                    forces.getCDaxial() // total Cd
            );
            lastPrintTime = status.getSimulationTime();
        }

        return forces;
    }
}