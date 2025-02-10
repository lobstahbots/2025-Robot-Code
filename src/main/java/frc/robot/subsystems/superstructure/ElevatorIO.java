package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double rightPosition;
        public double rightVelocity;
        public double rightAppliedVoltage;
        public double rightSupplyCurrent;
        public double rightStatorCurrent;
        public double rightTorqueCurrent;
        public double rightTempCelsius;
        public double leftPosition;
        public double leftVelocity;
        public double leftAppliedVoltage;
        public double leftSupplyCurrent;
        public double leftStatorCurrent;
        public double leftTorqueCurrent;
        public double leftTempCelsius;
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void setVoltage(double voltage);

    public void setPosition(double position); 
  
    public void stop();
    
    public void resetEncoder(double position);
  }
