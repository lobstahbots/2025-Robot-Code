// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.security.PermissionCollection;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax intakeRoller;
  private final SparkMax intakePivot;
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(int intakeRollerID, int intakePivotID) {
    intakeRoller = new SparkMax(intakeRollerID, MotorType.kBrushless);
    intakePivot = new SparkMax(intakePivotID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(20);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    intakeRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeRollerSpeed(double speed) {
    intakeRoller.set(speed);
  }

  public void setIntakePivotSpeed(double speed) {
    intakePivot.set(speed);
  }

  public void stopIntakeRoller() {
    intakeRoller.stopMotor();
  }

  public void stopIntakePivot() {
    intakePivot.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
