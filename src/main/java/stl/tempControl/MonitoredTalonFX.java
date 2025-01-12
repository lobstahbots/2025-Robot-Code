// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package stl.tempControl;

import com.ctre.phoenix6.hardware.TalonFX;

import stl.tempControl.TemperatureMonitor.Monitorable;

/** A temperature-controlled Talon FX motor controller. */
public class MonitoredTalonFX extends TalonFX implements Monitorable {
    private boolean disabled = false;
    private final String label;

    /**
     * Constructs a new temperature-controlled Talon FX motor controller object.
     * <p>
     * Constructs the device using the default CAN bus for the system:
     * <ul>
     *   <li>"rio" on roboRIO
     *   <li>"can0" on Linux
     *   <li>"*" on Windows
     * </ul>
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param label label The label associated with this motor, for use in {@link stl.networkalerts.Alert NetworkAlerts}.
     */
    public MonitoredTalonFX(int deviceId, String label) {
        super(deviceId);
        this.label = label;
    }

    /**
     * Constructs a new temperature-controlled Talon FX motor controller object default label "Motor deviceId",
     * where deviceId is the device ID.
     * <p>
     * Constructs the device using the default CAN bus for the system:
     * <ul>
     *   <li>"rio" on roboRIO
     *   <li>"can0" on Linux
     *   <li>"*" on Windows
     * </ul>
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     */
    public MonitoredTalonFX(int deviceID) {
        this(deviceID, String.format("Motor %d", deviceID));
    }

    public void setDisabled(boolean disable) {
        if (disable) super.set(0);
        disabled = disable;
    }
    
    public boolean getDisabled() {
        return disabled;
    }
    
    public void set(double speed) {
        if (!disabled) super.set(speed);
    }
    
    public void setVoltage(double outputVolts) {
        if (!disabled) super.setVoltage(outputVolts);
    }
    
    public String getLabel() {
        return label;
    }

    public double getMotorTemperature() {
        return getDeviceTemp().getValueAsDouble();
    }
}
