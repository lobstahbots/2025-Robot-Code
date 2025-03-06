package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.security.PermissionCollection;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
public class Ramp extends SubsystemBase{
    private final SparkMax motor;
    //creates new motor
    public Ramp(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.stopMotor();
      }

      @Override
    public void periodic() {
     // This method will be called once per scheduler run
    }
}
