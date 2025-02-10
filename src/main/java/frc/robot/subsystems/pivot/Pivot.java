// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private final SparkMax pivotMotor;
  private final PIDController pivotPID = new PIDController(PivotConstants.KP, PivotConstants.KI, PivotConstants.KD);
  private final DutyCycleEncoder encoder;
  private final ArmFeedforward feedForward = new ArmFeedforward(PivotConstants.KS, PivotConstants.KG, PivotConstants.KV, PivotConstants.KA);

  /** Creates a new Pivot. */
  public Pivot(int pivotMotorID, int encoderChannel) {
    SparkMaxConfig config = new SparkMaxConfig();
    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    config.smartCurrentLimit(PivotConstants.PIVOT_MOTOR_CURRNET_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = new DutyCycleEncoder(encoderChannel);
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public void setDesiredAngle(double desiredAngle) {
    double pidOutput = pivotPID.calculate(encoder.get(), desiredAngle);
    double feedForwardOutput = feedForward.calculate(encoder.get(), pivotMotor.getEncoder().getVelocity());
    pivotMotor.setVoltage(feedForwardOutput + pidOutput);
  }

  public void resetControllerError() {
    pivotPID.reset();
  }

  public void setIdleMode(IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode);
    pivotMotor.configure(config,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
