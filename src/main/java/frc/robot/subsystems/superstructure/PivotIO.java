package frc.robot.subsystems.superstructure;

public interface PivotIO {
    public static class PivotIOInputs {
        /**
         * Position in radians
         */
        public double position = 0.0;
        /**
         * Velocity in rad/sec
         */
        public double velocity = 0.0;
        /**
         * Supply current in amperes
         */
        public double supplyCurrent = 0.0;
        /**
         * Stator current in amperes
         */
        public double statorCurrent = 0.0;
        /**
         * Torque current in amperes
         */
        public double torqueCurrent = 0.0;
        /**
         * Temperature in degrees Celsius
         */
        public double temperature = 0.0;
        /**
         * Applied voltage in volts
         */
        public double appliedVoltage = 0.0;
    }

    public void updateInputs(PivotIOInputs inputs);

    /**
     * Set the voltage of the motors
     * @param voltage the voltage to set
     */
    public void setVoltage(double voltage);

    /**
     * Halt all pivot motion
     */
    public void stop();
}
