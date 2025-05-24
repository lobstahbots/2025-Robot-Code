package frc.robot.subsystems.endEffector.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    public static class CoralEndEffectorIOInputs {
       /**
         * Velocity of motor in rotations/sec
         */
        public double velocity = 0.0;
        /**
         * Applied voltage to motor in volts
         */
        public double appliedVoltage = 0.0;
        /**
         * Output current of motor in amperes
         */
        public double currentAmps = 0.0;
        /**
         * Temperature in degrees Celsius of motor
         */
        public double tempCelsius = 0.0;
        /**
         * Whether or not the beam on the beam break is broken.
         */
        public boolean beamBreakTriggered = false;
    }

    public default void updateInputs(CoralEndEffectorIOInputs inputs) {};

    /**
     * Halt motion of this end effector
     */
    public default void stopMotor() {};

    public default void setVoltage(double voltage) {};

    /**
     * Set the speed of the end effector
     * 
     * @param speed speed to set
     */
    public default void setSpeed(double speed) {};

    /** Enable or disable brake mode on the motors. */
    public default void setIdleMode(boolean isBrake) {};

    public default void periodic() {};
}
