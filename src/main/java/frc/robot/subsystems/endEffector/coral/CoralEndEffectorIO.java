package frc.robot.subsystems.endEffector.coral;
import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    static class CoralEndEffectorIOInputs {
        double appliedVoltage = 0.0;
        double currentAmps = 0.0;
    }

    public default void updateInputs(CoralEndEffectorIOInputs inputs) {}

    public default void stopMotor() {};

    public default void setSpeed(double speed) {};

    public default void periodic() {};
}
