// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.tempControl;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.tempControl.TemperatureMonitor.Monitorable;

/** A temperature-monitored SPARK MAX motor controller. */
public class MonitoredSparkMax extends SparkMax implements Monitorable {
  private boolean disabled = false;
  private final String label;
  
  /**
   * Create a new object to control a SPARK MAX motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   * @param label The label associated with this motor, for use in {@link frc.robot.util.networkalerts.Alert NetworkAlerts}.
   */
  public MonitoredSparkMax(int deviceId, MotorType type, String label) {
    super(deviceId, type);
    this.label = label;
  }

  /**
   * Create a new object to control a SPARK MAX motor Controller with default label "Motor deviceId",
   * where deviceId is the device ID.
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public MonitoredSparkMax(int deviceId, MotorType type) {
    this(deviceId, type, String.format("Motor %d", deviceId));
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
}
