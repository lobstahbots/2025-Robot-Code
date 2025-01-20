// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeIntake extends SubsystemBase {
  private final CANSparkMax intakeRoller;
  private final CANSparkMax intakePivot;
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(int intakeRollerID, int intakePivotID) {
    intakeRoller = new CANSparkMax(intakeRollerID, MotorType.kBrushless);
    intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    intakeRoller.setIdleMode(IdleMode.kBrake);
    intakePivot.setIdleMode(IdleMode.kBrake);
    intakeRoller.setSmartCurrentLimit(40);
    intakePivot.setSmartCurrentLimit(40);
    intakeRoller.setInverted(false);
    intakePivot.setInverted(false);
  }

  public void setIntakeRollerSpeed(double speed){
    intakeRoller.set(speed);
  }

  public void setIntakePivotSpeed(double speed){
    intakePivot.set(speed);
  }

  public void stopIntakeRoller(){
    intakeRoller.stopMotor();
  }

  public void stopIntakePivot(){
    intakePivot.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
