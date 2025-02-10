package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class EndEffector extends SubsystemBase {
  private final SparkMax endEffectorMotor;

  public EndEffector(int endEffectorMotorID) {
    endEffectorMotor = new SparkMax(endEffectorMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    endEffectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopMotor() {
    endEffectorMotor.stopMotor();
  }

  public void runMotor(double speed) {
    endEffectorMotor.set(speed);
  }
}