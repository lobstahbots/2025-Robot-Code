package frc.robot.subsystems.endEffector.coral;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface CoralEndEffectorIO {
    @AutoLog
    static class CoralEndEffectorIOInputs {
        Rotation2d position = new Rotation2d();
        double velocity = 0.0;
        double appliedVoltage = 0.0;
        double currentAmps = 0.0;
        double tempCelsius = 0.0;
    }

    public void updateInputs(CoralEndEffectorIOInputs inputs);

    public void stopMotor();

    public void setSpeed(double speed);

    public default void periodic() {};
}
