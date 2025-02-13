package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        /**
         * Position measured from the right motor in meters
         */
        public double rightPosition = 0.0;
        /**
         * Velocity measured from the right motor in m/s
         */
        public double rightVelocity = 0.0;
        /**
         * Voltage applied to the right motor in volts
         */
        public double rightAppliedVoltage = 0.0;
        /**
         * Supply current of the right motor in amperes
         */
        public double rightSupplyCurrent = 0.0;
        /**
         * Stator current of the right motor in amperes
         */
        public double rightStatorCurrent = 0.0;
        /**
         * Torque current of the right motor in amperes
         */
        public double rightTorqueCurrent = 0.0;
        /**
         * Temperature of the right motor in degrees Celsius
         */
        public double rightTempCelsius = 0.0;
        /**
         * Position measured from the left motor in meters
         */
        public double leftPosition = 0.0;
        /**
         * Velocity measured from the left motor in m/s
         */
        public double leftVelocity = 0.0;
        /**
         * Voltage applied to the left motor in volts
         */
        public double leftAppliedVoltage = 0.0;
        /**
         * Supply current of the left motor in amperes
         */
        public double leftSupplyCurrent = 0.0;
        /**
         * Stator current of the left motor in amperes
         */
        public double leftStatorCurrent = 0.0;
        /**
         * Torque current of the left motor in amperes
         */
        public double leftTorqueCurrent = 0.0;
        /**
         * Temperature of the left motor in degrees Celcius
         */
        public double leftTempCelsius = 0.0;
        /**
         * Whether or not the limit switch is hit
         */
        public boolean limitSwitchHit = false;
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
