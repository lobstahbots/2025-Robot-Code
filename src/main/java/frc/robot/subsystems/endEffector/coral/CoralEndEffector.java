package frc.robot.subsystems.endEffector.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralEndEffectorConstants;
import frc.robot.subsystems.endEffector.coral.CoralEndEffectorIO.CoralEndEffectorIOInputs;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralEndEffector extends SubsystemBase {
  private final CoralEndEffectorIOInputsAutoLogged inputs = new CoralEndEffectorIOInputsAutoLogged();
  private final CoralEndEffectorIO io;

  public CoralEndEffector(CoralEndEffectorIO io) {
    this.io = io;
  }

  public void stopMotor() {
    io.stopMotor();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralEndEffector", inputs);
  }
}