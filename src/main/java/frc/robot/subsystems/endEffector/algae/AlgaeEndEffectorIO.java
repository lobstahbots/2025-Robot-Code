package frc.robot.subsystems.endEffector.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeEndEffectorIO {
    @AutoLog
    public static class AlgaeEndEffectorIOInputs {
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
    }

    public void updateInputs(AlgaeEndEffectorIOInputs inputs);

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
