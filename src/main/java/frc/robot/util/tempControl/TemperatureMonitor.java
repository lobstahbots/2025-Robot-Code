// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.tempControl;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.TempConstants;

/**
 * Class to monitor the temperature of some items.
 */
public class TemperatureMonitor {
    /**
     * A monitorable device, likely a motor. Needs to be able to save its disabled state, get
     * its disabled state, and get its temperature and label.
     */
    public static interface Monitorable {
        /**
         * Sets the disabled state of the monitorable.
         * @param disable Whether or not to disable.
         */
        public void setDisabled(boolean disable);
        /**
         * Gets the disabled state of the monitorable.
         * @return Whether or not it is disabled.
         */
        public boolean getDisabled();
        /**
         * Gets the motor temperature.
         * @return The temperature.
         */
        public double getMotorTemperature();
        /**
         * Gets the label for this.
         * @return The label.
         */
        public String getLabel();
    }
    private double overheatTime = -1;
    private boolean disabled = false;
    private List<Monitorable> motors;
    private final HashMap<String, Alert> alerts;
    private final boolean disableAll;

    /**
     * Create a TemperatureMonitor to monitor some {@link Monitorable}s.
     * @param motors A list of {@link Monitorable}s to monitor
     * @param disableAll Whether or not to disable all the motors when just one
     *      of them overheats.
     */
    public TemperatureMonitor(List<Monitorable> motors, boolean disableAll) {
        this.motors = motors;
        alerts = new HashMap<>(motors.size(), 1); // Load factor can be 1 because the size will never change
        for (var motor : motors) {
            alerts.put(motor.getLabel(), new Alert(
                String.format(
                    "%s has overheated above %d C. It and related motors will be disabled until its temperature drops below %d C.",
                    motor.getLabel(),
                    TempConstants.OVERHEAT_TEMP,
                    TempConstants.SAFE_TEMP
                ),
                AlertType.kError
            ));
        }
        this.disableAll = disableAll;
    }

    /**
     * Create a TemperatureMonitor to monitor some {@link Monitorable}s,
     * disabling all of them when one overheats.
     * @param motors A list of {@link Monitorable}s to monitor
     */
    public TemperatureMonitor(List<Monitorable> motors) {
        this(motors, true);
    }

    /**
     * Monitor the {@link Monitorable}s. This method should be called periodically,
     * probably in the <code>periodic</code> method of a subsystem.
     */
    public void monitor() {
        boolean safe = true;
        for (Monitorable motor : motors) {
            double temp = motor.getMotorTemperature();
            if (temp > TempConstants.SAFE_TEMP) safe = false;
            else {
                alerts.get(motor.getLabel()).set(false);
                if (!disableAll) motor.setDisabled(false); // This motor is at a safe temperature so if we don't disable all motors we know it's safe to enable
            }
            if (temp > TempConstants.OVERHEAT_TEMP && overheatTime == -1) overheatTime = Timer.getFPGATimestamp();
            else if (temp > TempConstants.OVERHEAT_TEMP && Timer.getFPGATimestamp() - overheatTime >= 2) {
                disabled = true;
                alerts.get(motor.getLabel()).set(true);
                motor.setDisabled(true); // It's overheated so we definitely disable it
            }
            if (disabled && disableAll) motor.setDisabled(true);
            else if (!disabled) motor.setDisabled(false);
        }
        if (safe) {
            disabled = false;
            overheatTime = -1;
        }
    }
}
