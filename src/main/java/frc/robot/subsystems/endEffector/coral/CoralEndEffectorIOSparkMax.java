package frc.robot.subsystems.endEffector.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CoralEndEffectorConstants;

public class CoralEndEffectorIOSparkMax implements CoralEndEffectorIO {
  private final SparkMax coralEndEffectorMotor;
  private final RelativeEncoder encoder;

  public CoralEndEffectorIOSparkMax(int coralEndEffectorMotorID, int coralEndEffectorEncoderChannel) {
    this.coralEndEffectorMotor = new SparkMax(coralEndEffectorMotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CoralEndEffectorConstants.CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    coralEndEffectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = coralEndEffectorMotor.getEncoder();
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
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVoltage = coralEndEffectorMotor.getAppliedOutput() * coralEndEffectorMotor.getBusVoltage();
    inputs.currentAmps = coralEndEffectorMotor.getOutputCurrent();
    inputs.tempCelsius = coralEndEffectorMotor.getMotorTemperature();
  }

  @Override
  public void periodic() {
  }
}
