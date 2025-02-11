package frc.robot.subsystems.endEffector.coral;
import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    public static class CoralEndEffectorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public void updateInputs(CoralEndEffectorIOInputs inputs);

    public void stopMotor();

    public void setSpeed(double speed);

    public default void periodic() {};
}
