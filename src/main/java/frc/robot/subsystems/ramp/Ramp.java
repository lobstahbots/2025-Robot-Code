// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;

import frc.robot.Constants.RampConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Ramp extends SubsystemBase {
  /** Creates a new ramp. */
  private final SparkMax rampMotor;
  private final RelativeEncoder encoder; 

  public Ramp(int id) {
    this.rampMotor = new SparkMax(id, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.setSmartCurrentLimit(RampConstants.CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.encoder.velocityConversionFactor(1.0);

    rampMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = rampMotor.getEncoder();
  }

  public void stopMotor() {
    rampMotor.stopMotor();
  }

  public void setSpeed(double speed){
    rampMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
