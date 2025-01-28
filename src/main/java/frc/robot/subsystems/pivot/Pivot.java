// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private final SparkMax pivotMotor;
  private final PIDController pivotPID = new PIDController(PivotConstants.P_CONTROLLER, PivotConstants.I_CONTROLLER, PivotConstants.D_CONTROLLER);
  private final DutyCycleEncoder encoder;
  private final ArmFeedforward feedForward = new ArmFeedforward(0, 0, 0);

  /** Creates a new Pivot. */
  public Pivot(int pivotMotorID, int encoderChanel) {
    SparkMaxConfig config = new SparkMaxConfig();
    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = new DutyCycleEncoder(encoderChanel);
  }

  public void stopPivot(){
    pivotMotor.stopMotor();
  }

  public void setDesiredAngle(double desiredAngle){
    pivotPID.setSetpoint(desiredAngle);
    double pidOutput = pivotPID.calculate(encoder.get(), desiredAngle);
    double feedForwardOutput = feedForward.calculate(encoder.get(), pivotMotor.getEncoder().getVelocity());
    pivotMotor.setVoltage(feedForwardOutput + pidOutput);
  }

  public void resetControllerError(){
    pivotPID.reset();
  }

  public void setIdleMode(IdleMode idleMode){
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode);
    pivotMotor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(double voltage){
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
