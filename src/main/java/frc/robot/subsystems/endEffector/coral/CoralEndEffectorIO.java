package frc.robot.subsystems.endEffector;
import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    static class CoralEndEffectorIOInputs {
        double position = 0.0;
        double velocity = 0.0;
        double appliedVoltage = 0.0;
        double currentAmps = 0.0;
    }

    public default void updateInputs(CoralEndEffectorIOInputs inputs) {}

    public default void stopMotor() {};

    public default void runMotor(double speed) {};
}
