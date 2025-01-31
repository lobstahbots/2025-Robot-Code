package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    static class ElevatorIOInputs {
        double rightPosition;
        double rightVelocity;
        double rightAppliedVoltage;
        double rightSupplyCurrent;
        double rightTorqueCurrent;
        double rightTempCelsius;
        double leftPosition;
        double leftVelocity;
        double leftAppliedVoltage;
        double leftSupplyCurrent;
        double leftTorqueCurrent;
        double leftTempCelsius;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void stop() {}
    
}
