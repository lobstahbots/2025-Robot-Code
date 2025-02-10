package frc.robot.subsystems.endEffector.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CoralEndEffectorConstants;

public class CoralEndEffectorIOSparkMax implements CoralEndEffectorIO {
  private final SparkMax coralEndEffectorMotor;

  public CoralEndEffectorIOSparkMax(int coralEndEffectorMotorID) {
    this.coralEndEffectorMotor = new SparkMax(coralEndEffectorMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CoralEndEffectorConstants.MAX_CURRENT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    coralEndEffectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void stopMotor() {
    coralEndEffectorMotor.stopMotor();
  }

  @Override
  public void setSpeed(double speed) {
    coralEndEffectorMotor.set(speed);
  }

  @Override
  public void updateInputs(CoralEndEffectorIOInputs inputs) {
    inputs.appliedVoltage = coralEndEffectorMotor.getAppliedOutput() * coralEndEffectorMotor.getBusVoltage();
    inputs.currentAmps = coralEndEffectorMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
  }
}
