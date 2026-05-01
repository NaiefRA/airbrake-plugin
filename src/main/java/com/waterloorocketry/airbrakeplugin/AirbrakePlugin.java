package com.waterloorocketry.airbrakeplugin;

import com.waterloorocketry.airbrakeplugin.airbrake.Airbrakes;

import com.waterloorocketry.airbrakeplugin.airbrake.SimulatedAirbrakes;
import com.waterloorocketry.airbrakeplugin.controller.Controller;
import com.waterloorocketry.airbrakeplugin.controller.PIDController;
import com.waterloorocketry.airbrakeplugin.controller.SMCController;
// import com.waterloorocketry.airbrakeplugin.simulated.Noise;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.unit.UnitGroup;

import java.util.ArrayList;
import java.util.List;

/**
 * Initialize the plugin.
 */
public class AirbrakePlugin extends AbstractSimulationExtension {
    @Override
    public String getName() {
        return "Airbrakes";
    }

    // Create new FlightDataType to hold airbrake extension percentage
    private static final FlightDataType airbrakeExt = FlightDataType.getType("airbrakeExt", "airbrakeExt",
            UnitGroup.UNITS_RELATIVE);
    // Create new FlightDataType to hold trajpred apogee output even though its not
    // a flight data, this allows us to graph it
    private static final FlightDataType predictedApogee = FlightDataType.getType("predictedApogee", "predictedApogee",
            UnitGroup.UNITS_DISTANCE);
    private static final ArrayList<FlightDataType> types = new ArrayList<FlightDataType>();

    /**
     * Initialize the new airbrakeExt datatype we created by returning it here
     * 
     * @return
     */
    @Override
    public List<FlightDataType> getFlightDataTypes() {
        return types;
    }

    AirbrakePlugin() {
        types.add(airbrakeExt);
        types.add(predictedApogee);
    }

    /**
     * Initialize this extension before simulations by adding the simulation
     * listener.
     * 
     * @param conditions
     * @throws SimulationException
     */
    @Override
    public void initialize(SimulationConditions conditions) throws SimulationException {
        Controller controller;

        controller = new PIDController((float) getTargetApogee(), (float) getKp(), (float) getKi(), (float) getKd(),
                (float) getISatmax());

        controller = new SMCController((float) getTargetApogee(), (float) getC(), (float) getK_smc(), (float) getPhi());

        Airbrakes airbrakes = new SimulatedAirbrakes();

        conditions.getSimulationListenerList()
                .add(new AirbrakePluginSimulationListener(airbrakes, controller, getExtTime(), getRateLimit()));
    }

    //
    // Getter/setters for all the values that are user-adjustable via the plugin
    // config panel
    // The setters are used indirectly in AirbrakePluginConfigurator
    //

    public double getTargetApogee() {
        return config.getDouble("targetApogee", 3048.0);
    }

    public void setTargetApogee(double targetApogee) {
        config.put("targetApogee", targetApogee);
        fireChangeEvent();
    }

    public double getKp() {
        return config.getDouble("Kp", 0.001424);
    }

    public void setKp(double Kp) {
        config.put("Kp", Kp);
        fireChangeEvent();
    }

    public double getKi() {
        return config.getDouble("Ki", 0.0008);
    }

    public void setKi(double Ki) {
        config.put("Ki", Ki);
        fireChangeEvent();
    }

    public double getKd() {
        return config.getDouble("Kd", 0.0);
    }

    public void setKd(double Kd) {
        config.put("Kd", Kd);
        fireChangeEvent();
    }

    public double getISatmax() {
        return config.getDouble("ISatmax", 10.0);
    }

    public void setISatmax(double v) {
        config.put("ISatmax", v);
        fireChangeEvent();
    }

    public double getExtTime() {
        return config.getDouble("ExtTime", 8.0);
    }

    public void setExtTime(double time) {
        config.put("ExtTime", time);
        fireChangeEvent();
    }

    public void setRateLimit(double time) {
        config.put("RateLimit", time);
        fireChangeEvent();
    }

    public double getRateLimit() {
        return config.getDouble("RateLimit", 0.033);
    }

    public double getK_smc() {
        return config.getDouble("K_smc", 1.0);
    }

    public void setK_smc(double K_smc) {
        config.put("K_smc", K_smc);
        fireChangeEvent();
    }

    public double getC() {
        return config.getDouble("C", 0.4);
    }

    public void setC(double C) {
        config.put("C", C);
        fireChangeEvent();
    }

    public double getPhi() {
        return config.getDouble("Phi", 1.0);
    }

    public void setPhi(double Phi) {
        config.put("Phi", Phi);
        fireChangeEvent();
    }
}