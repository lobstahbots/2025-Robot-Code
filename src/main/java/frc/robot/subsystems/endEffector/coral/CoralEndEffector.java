// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector.coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralEndEffector extends SubsystemBase {
  private final SparkMax coralEndEffectorMotor;
  /** Creates a new CoralEndEffector. */
  public CoralEndEffector(int coralEndEffectorMotorID) {
    this.coralEndEffectorMotor = new SparkMax (coralEndEffectorMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(20);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    coralEndEffectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopMotors() {
    coralEndEffectorMotor.stopMotor();
  }

  public void setSpeed(double speed) {
    this.coralEndEffectorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
