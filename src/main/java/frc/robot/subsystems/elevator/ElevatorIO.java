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

    public void updateInputs(ElevatorIOInputs inputs);

    public void setVoltage(double voltage);

    public void setPosition(double position); 
  
    public void stop();
    
    public void resetEncoder(double position);
  }
