package com.waterloorocketry.airbrakeplugin;

import com.waterloorocketry.airbrakeplugin.airbrake.Airbrakes;
import com.waterloorocketry.airbrakeplugin.controller.Controller;
import com.waterloorocketry.airbrakeplugin.controller.TrajectoryPrediction;
// import com.waterloorocketry.airbrakeplugin.simulated.Noise;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
// import net.sf.openrocket.simulation.FlightEvent;
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
    public static final FlightDataType airbrakeExtDataType = FlightDataType.getType("airbrakeExt", "airbrakeExt",
            UnitGroup.UNITS_RELATIVE);
    public static final FlightDataType predictedApogeeDataType = FlightDataType.getType("predictedApogee",
            "predictedApogee", UnitGroup.UNITS_DISTANCE);
    private double ext = 0.0;
    private final double extTime;
    private final double rateLimit;

    public AirbrakePluginSimulationListener(Airbrakes airbrakes, Controller controller, double extTime,
            double rateLimit) {
        super();
        this.airbrakes = airbrakes;
        this.controller = controller;
        this.extTime = extTime;
        this.rateLimit = rateLimit;
    }

    // Airbrakes only allowed between the given time (default 9 s) and while
    // vertical velocity > 34 m/s
    private boolean isExtensionAllowed(SimulationStatus status) {
        return status.getSimulationTime() > extTime && status.getRocketVelocity().z > 15.0;
    }

    /**
     * Runs before each timestep.
     */
    @Override
    public boolean preStep(SimulationStatus status) {
        FlightDataBranch flightData = status.getFlightData();
        Controller.RocketState data = new Controller.RocketState(status);

        // Only run controller during coast phase.
        if (isExtensionAllowed(status)) {
            ext = controller.calculateTargetExt(data, status.getSimulationTime(), ext, rateLimit);
            if (!(0.0 <= ext && ext <= 1.0)) {
                throw new IndexOutOfBoundsException("airbrakes extension amount was not from 0 to 1: " + ext);
            }
        } else {
            // Graceful retraction respecting the rate limit
            ext = Math.max(0.0, ext - rateLimit);
        }

        flightData.setValue(airbrakeExtDataType, ext);

        // This is solely for graphing trajectory prediction outputs
        flightData.setValue(predictedApogeeDataType, TrajectoryPrediction.get_max_altitude(data));

        return true;
    }

    /**
     * Flight conditions for the current timestep.
     */
    private FlightConditions flightConditions = null;

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

        double extension = status.getFlightData().getLast(airbrakeExtDataType);
        double rocketCd = forces.getCDaxial();
        double totalCD = 0;

        // Apply drag whenever airbrakes are physically extended,
        // even if the controller is retracting them due to low velocity
        if (extension > 0.0) {
            final double velocityZ = status.getRocketVelocity().z;
            final double altitude = status.getRocketPosition().z
                    + status.getSimulationConditions().getLaunchSite().getAltitude();

            totalCD = airbrakes.calculateDragCoefficient(extension, velocityZ, altitude);
            forces.setCDaxial(totalCD);
        }

        // printing
        if (status.getSimulationTime() - lastPrintTime > 0.2) {
            System.out.printf(
                    "Time: %6.2fs | Extended: %-5b | Ext: %3.0f%% | Rocket Cd: %6.4f | Total Cd: %6.4f%n",
                    status.getSimulationTime(), // Time
                    extension > 0, // Whether or not airbrakes are extended (True/False)
                    extension * 100, // Extension percentage (0-100%)
                    rocketCd, // OR Cd
                    forces.getCDaxial() // total Cd
            );
            lastPrintTime = status.getSimulationTime();
        }

        return forces;
    }
}