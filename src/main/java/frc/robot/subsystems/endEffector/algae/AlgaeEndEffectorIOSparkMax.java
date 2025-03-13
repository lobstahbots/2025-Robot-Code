package frc.robot.subsystems.endEffector.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeEndEffectorConstants;

public class AlgaeEndEffectorIOSparkMax implements AlgaeEndEffectorIO {
  private final SparkMax algaeMotor;
  private final RelativeEncoder encoder;

  public AlgaeEndEffectorIOSparkMax(int id) {
    this.algaeMotor = new SparkMax(id, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(AlgaeEndEffectorConstants.CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.encoder.velocityConversionFactor(1.0 / 60);
    algaeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = algaeMotor.getEncoder();
  }

  @Override
  public void stopMotor() {
    algaeMotor.stopMotor();
  }

  @Override
  public void setSpeed(double speed) {
    algaeMotor.set(speed);
  }

  @Override
  public void updateInputs(AlgaeEndEffectorIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVoltage = algaeMotor.getAppliedOutput() * algaeMotor.getBusVoltage();
    inputs.currentAmps = algaeMotor.getOutputCurrent();
    inputs.tempCelsius = algaeMotor.getMotorTemperature();
  }

  @Override
  public void setIdleMode(boolean isBrake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    algaeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
