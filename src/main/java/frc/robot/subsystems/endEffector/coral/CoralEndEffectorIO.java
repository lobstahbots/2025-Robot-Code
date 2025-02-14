package frc.robot.subsystems.endEffector.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {
    @AutoLog
    public static class CoralEndEffectorIOInputs {
        /**
         * Velocity of left motor in rotations/sec
         */
        public double leftVelocity = 0.0;
        /**
         * Applied voltage to left motor in volts
         */
        public double leftAppliedVoltage = 0.0;
        /**
         * Output current of left motor in amperes
         */
        public double leftCurrentAmps = 0.0;
        /**
         * Temperature in degrees Celsius of left motor
         */
        public double leftTempCelsius = 0.0;

        /**
         * Velocity of right motor in rotations/sec
         */
        public double rightVelocity = 0.0;
        /**
         * Applied voltage to right motor in volts
         */
        public double rightAppliedVoltage = 0.0;
        /**
         * Output current of right motor in amperes
         */
        public double rightCurrentAmps = 0.0;
        /**
         * Temperature in degrees Celsius of right motor
         */
        public double rightTempCelsius = 0.0;
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

    /**
     * Set the speed of the end effector
     * 
     * @param leftSpeed  speed for left motor
     * @param rightSpeed speed for right motor
     */
    public void setSpeed(double leftSpeed, double rightSpeed);

    public default void periodic() {};
}
