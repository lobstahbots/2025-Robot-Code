package frc.robot.subsystems.otbCoral;

import edu.wpi.first.math.geometry.Rotation2d;

public class OTBIntakeState {
    
    /**
     * Gives the rotation of the first pivot joint(one directly attached to drivebase) in radians
     */
    public final Rotation2d joint1MotorRotation;

    /** 
     * Gives the rotation of the second pivot joint in radians
    */
    public final Rotation2d joint2MotorRotation;

    /** 
     * Gives the velocity of the first pivot joint in radians/second
    */
    public final double joint1MotorVelocity;

    /**
     * Gives the velocity of the second pivot joint in radians/second
    */
    public final double joint2MotorVelocity;

    public OTBIntakeState (Rotation2d joint1MotorRotation, Rotation2d joint2MotorRotation, double joint1MotorVelocity, double joint2MotorVelocity) {
        this.joint1MotorRotation = joint1MotorRotation;
        this.joint2MotorRotation = joint2MotorRotation;
        this.joint1MotorVelocity = joint1MotorVelocity;
        this.joint2MotorVelocity = joint2MotorVelocity;
    }

}
