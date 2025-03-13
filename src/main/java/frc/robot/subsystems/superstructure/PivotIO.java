package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        /**
         * Position in a {@link Rotation2d}
         */
        public Rotation2d position = new Rotation2d();
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

    /** Enable or disable brake mode on the motors. */
    public void setIdleMode(boolean isBrake);
}
