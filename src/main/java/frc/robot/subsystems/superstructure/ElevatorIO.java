package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        /**
         * Position measured from the right motor in meters
         */
        public double rightPosition;
        /**
         * Velocity measured from the right motor in m/s
         */
        public double rightVelocity;
        /**
         * Voltage applied to the right motor in volts
         */
        public double rightAppliedVoltage;
        /**
         * Supply current of the right motor in amperes
         */
        public double rightSupplyCurrent;
        /**
         * Stator current of the right motor in amperes
         */
        public double rightStatorCurrent;
        /**
         * Torque current of the right motor in amperes
         */
        public double rightTorqueCurrent;
        /**
         * Temperature of the right motor in degrees Celsius
         */
        public double rightTempCelsius;
        /**
         * Position measured from the left motor in meters
         */
        public double leftPosition;
        /**
         * Velocity measured from the left motor in m/s
         */
        public double leftVelocity;
        /**
         * Voltage applied to the left motor in volts
         */
        public double leftAppliedVoltage;
        /**
         * Supply current of the left motor in amperes
         */
        public double leftSupplyCurrent;
        /**
         * Stator current of the left motor in amperes
         */
        public double leftStatorCurrent;
        /**
         * Torque current of the left motor in amperes
         */
        public double leftTorqueCurrent;
        /**
         * Temperature of the left motor in degrees Celcius
         */
        public double leftTempCelsius;
    }

    public void updateInputs(ElevatorIOInputs inputs);

    /**
     * Set the voltage of the elevator motors for SysId.
     * 
     * @param voltage the voltage to set
     */
    public void setVoltage(double voltage);

    /**
     * Set the internal PIDF loop to aim for this position.
     * 
     * @param position the position setpoint
     */
    public void setPosition(double position);

    /**
     * Stop all elevator motion
     */
    public void stop();

    /**
     * Reset the internal encoder to a specified position
     * 
     * @param position the position to reset to
     */
    public void resetEncoder(double position);
}
