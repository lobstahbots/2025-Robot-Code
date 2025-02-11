package frc.robot.subsystems.endEffector.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    public static class CoralEndEffectorIOInputs {
        /**
         * Position in rotations
         */
        public double position = 0.0;
        /**
         * Velocity in rotations/sec
         */
        public double velocity = 0.0;
        /**
         * Applied voltage in volts
         */
        public double appliedVoltage = 0.0;
        /**
         * Output current in amperes
         */
        public double currentAmps = 0.0;
        /**
         * Temperature in degrees Celsius
         */
        public double tempCelsius = 0.0;
    }

    public void updateInputs(CoralEndEffectorIOInputs inputs);

    /**
     * Halt motion of this end effector
     */
    public void stopMotor();

    /**
     * Set the speed of the end effector
     * 
     * @param speed speed to set
     */
    public void setSpeed(double speed);

    public default void periodic() {};
}
