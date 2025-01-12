// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  /** Creates a new SwerveModuleSim. */
  private final SwerveModuleSimulation moduleSimulation;
  // private final Simulated
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController angleMotor;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private Rotation2d angularOffset;

  public SwerveModuleIOSim(double angularOffsetDegrees, SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
    driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT));
    angleMotor = moduleSimulation.useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT));
  }

  public void updateInputs(ModuleIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setDriveVoltage(0);
      setTurnVoltage(0);
    }

    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing().plus(angularOffset);
    inputs.turnPosition = new Rotation2d(moduleSimulation.getSteerRelativeEncoderPosition());
    inputs.drivePosition = new Rotation2d(moduleSimulation.getDriveWheelFinalPosition()
        .plus(moduleSimulation.getDriveWheelFinalSpeed().times(Seconds.of(SimConstants.LOOP_TIME))));
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = moduleSimulation.getDriveMotorSupplyCurrent().in(Amps);
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = moduleSimulation.getSteerMotorSupplyCurrent().in(Amps);
    inputs.angularOffset = angularOffset;
  }

  /**
   * Sets voltage of driving motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
  }

  /**
   * Sets voltage of turn motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    angleMotor.requestVoltage(Volts.of(turnAppliedVolts));
  }
}
