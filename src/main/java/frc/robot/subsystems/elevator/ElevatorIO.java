package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    static class ElevatorIOInputs {
        double rightPosition;
        double rightVelocity;
        double rightAppliedVoltage;
        double rightSupplyCurrent;
        double rightStatorCurrent;
        double rightTorqueCurrent;
        double rightTempCelsius;
        double leftPosition;
        double leftVelocity;
        double leftAppliedVoltage;
        double leftSupplyCurrent;
        double leftStatorCurrent;
        double leftTorqueCurrent;
        double leftTempCelsius;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(double voltage) {};

    public default void setPosition(double position) {}; 
  
    public default void stop() {};
    
    public default void resetEncoder(double position) {};
  }
