package frc.robot.subsystems.endEffector.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralEndEffectorConstants;

public class CoralEndEffectorIOSparkMax implements CoralEndEffectorIO {
  private final SparkMax coralMotor;
  private final RelativeEncoder encoder;

  public CoralEndEffectorIOSparkMax(int id) {
    this.coralMotor = new SparkMax(id, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CoralEndEffectorConstants.CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.encoder.velocityConversionFactor(1.0 / 60);
    coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = coralMotor.getEncoder();
  }

  @Override
  public void stopMotor() {
    coralMotor.stopMotor();
  }

  @Override
  public void setSpeed(double speed) {
    coralMotor.set(speed);
  }

  @Override
  public void updateInputs(CoralEndEffectorIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVoltage = coralMotor.getAppliedOutput() * coralMotor.getBusVoltage();
    inputs.currentAmps = coralMotor.getOutputCurrent();
    inputs.tempCelsius = coralMotor.getMotorTemperature();
  }
}
