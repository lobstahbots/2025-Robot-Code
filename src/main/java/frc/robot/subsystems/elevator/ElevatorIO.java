package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    static class ElevatorIOInputs {
        double rightPosition = 0.0;
        double rightVelocity = 0.0;
        double rightAppliedVoltage = 0.0;
        double rightSupplyCurrent = 0.0;
        double rightStatorCurrent = 0.0;
        double rightTorqueCurrent = 0.0;
        double rightTempCelsius = 0.0;
        double leftPosition = 0.0;
        double leftVelocity = 0.0;
        double leftAppliedVoltage = 0.0;
        double leftSupplyCurrent = 0.0;
        double leftStatorCurrent = 0.0;
        double leftTorqueCurrent = 0.0;
        double leftTempCelsius = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setLimitSwitches() {};

    public default void setVoltage(double voltage) {};

    public default void setPosition(double position) {}; 
  
    public default void stop() {};
    
    public default void resetEncoder(double position) {};
  }
