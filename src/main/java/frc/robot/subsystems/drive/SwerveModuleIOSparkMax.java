// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Arrays;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
  private final MonitoredSparkMax angleMotor;
  private final MonitoredSparkMax driveMotor;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder angleAbsoluteEncoder;
  private Rotation2d angularOffset;
  private final int moduleID;
  private final TemperatureMonitor monitor;

  /**
   * Creates a new SwerveModule for real cases.
   * 
   * @param moduleID             The module number (0-3).
   * @param angleMotorID         The CAN ID of the motor controlling the angle.
   * @param driveMotorID         The CAN ID of the motor controlling drive speed.
   * @param angularOffsetDegrees The offset angle in degrees.
   */
  public SwerveModuleIOSparkMax(int moduleID, String name, int angleMotorID, int driveMotorID,
      double angularOffsetDegrees, boolean inverted) {
    this.moduleID = moduleID;

    this.angleMotor = new MonitoredSparkMax(angleMotorID, MotorType.kBrushless, name + " angle motor");
    this.driveMotor = new MonitoredSparkMax(driveMotorID, MotorType.kBrushless, name + " drive motor");

    var driveEncoderConfig = new EncoderConfig()
        .positionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    var driveMotorConfig = new SparkMaxConfig().apply(driveEncoderConfig)
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT).idleMode(IdleMode.kBrake).voltageCompensation(12)
        .inverted(false);
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var angleMotorConfig = new SparkMaxConfig().smartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake).voltageCompensation(12).inverted(inverted);
    angleMotorConfig.absoluteEncoder
        .positionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drivingEncoder = driveMotor.getEncoder();
    angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder();

    monitor = new TemperatureMonitor(Arrays.asList(driveMotor, angleMotor));

    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
    drivingEncoder.setPosition(0);
    resetEncoders();
  }

  /** Stops the module motors. */
  public void stopMotors() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current encoder velocities of the module as a
   *         {@link SwerveModuleState}.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
  }

  /**
   * Returns the module ID.
   *
   * @return The ID number of the module (0-3).
   */
  public int getModuleID() {
    return moduleID;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current encoder positions position of the module as a
   *         {@link SwerveModulePosition}.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivingEncoder.getPosition(),
        new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
  }

  /**
   * Sets the braking mode of the driving motor.
   * 
   * @param mode the {@link IdleMode} to set motors to.
   */
  public void setDriveIdleMode(IdleMode mode) {
    driveMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the braking mode of the turning motor.
   * 
   * @param mode the {@link IdleMode} to set motors to.
   */
  public void setTurnIdleMode(IdleMode mode) {
    angleMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Zeroes the drive encoder. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  /**
   * Sets voltage of driving motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**
   * Sets voltage of turn motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setTurnVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePosition = driveMotor.getLastError() == REVLibError.kOk
        ? Rotation2d.fromRotations(drivingEncoder.getPosition())
        : inputs.drivePosition;
    inputs.driveVelocityRadPerSec = driveMotor.getLastError() == REVLibError.kOk
        ? Units.rotationsToRadians(drivingEncoder.getVelocity())
        : inputs.driveVelocityRadPerSec;
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnAbsolutePosition = angleMotor.getLastError() == REVLibError.kOk
        ? Rotation2d.fromRadians(-angleAbsoluteEncoder.getPosition() - angularOffset.getRadians())
        : inputs.turnPosition;
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec = angleMotor.getLastError() == REVLibError.kOk ? angleAbsoluteEncoder.getVelocity()
        : inputs.turnVelocityRadPerSec;
    inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.turnCurrentAmps = angleMotor.getOutputCurrent();
    inputs.angularOffset = angularOffset;
  }

  public void periodic() {
    monitor.monitor();
  }
}
