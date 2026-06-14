package com.waterloorocketry.airbrakeplugin;

import com.waterloorocketry.airbrakeplugin.airbrake.Airbrakes;
import com.waterloorocketry.airbrakeplugin.controller.Controller;
import com.waterloorocketry.airbrakeplugin.controller.TrajectoryPrediction;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.unit.UnitGroup;

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

    private boolean isExtensionAllowed(SimulationStatus status) {
        return status.getSimulationTime() > extTime && status.getRocketVelocity().z > 15.0;
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        FlightDataBranch flightData = status.getFlightData();
        Controller.RocketState data = new Controller.RocketState(status);

        if (isExtensionAllowed(status)) {
            ext = controller.calculateTargetExt(data, status.getSimulationTime(), ext, rateLimit);
            if (!(0.0 <= ext && ext <= 1.0)) {
                throw new IndexOutOfBoundsException("airbrakes extension amount was not from 0 to 1: " + ext);
            }
        } else {
            ext = Math.max(0.0, ext - rateLimit);
        }

        flightData.setValue(airbrakeExtDataType, ext);

        // --- FIXED PREDICTION CALL FOR GRAPHING ---
        double currentAlt = status.getRocketPosition().z
                + status.getSimulationConditions().getLaunchSite().getAltitude();

        double vX = status.getRocketVelocity().x;
        double vY = status.getRocketVelocity().y;
        double vZ = status.getRocketVelocity().z;
        double vMag = Math.sqrt(vX * vX + vY * vY + vZ * vZ);

        double currentInclDeg = 0.0;
        if (vMag > 0.0) {
            currentInclDeg = Math.toDegrees(Math.acos(vZ / vMag));
        }

        double currentDefDeg = ext * 60.0; // Assuming max is 60 deg

        double predictedApo = currentAlt
                + TrajectoryPrediction.get_apogee_delta(currentAlt, vMag, currentDefDeg, currentInclDeg);
        flightData.setValue(predictedApogeeDataType, predictedApo);

        return true;
    }

    private FlightConditions flightConditions = null;

    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions)
            throws SimulationException {
        this.flightConditions = flightConditions;
        return flightConditions;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus status, AerodynamicForces forces)
            throws SimulationException {

        double extension = status.getFlightData().getLast(airbrakeExtDataType);
        double totalCD = 0;

        if (extension > 0.0) {
            final double velocityZ = status.getRocketVelocity().z;
            final double altitude = status.getRocketPosition().z
                    + status.getSimulationConditions().getLaunchSite().getAltitude();

            totalCD = airbrakes.calculateDragCoefficient(extension, velocityZ, altitude);
            forces.setCDaxial(totalCD);
        }

        if (status.getSimulationTime() - lastPrintTime > 0.2) {
            lastPrintTime = status.getSimulationTime();
        }

        return forces;
    }
}