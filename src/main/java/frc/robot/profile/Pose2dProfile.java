package frc.robot.profile;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Pose2dProfile {
    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    /**
     * Create a new Pose2dProfile for motion profiling movement.
     * 
     * @param translationConstraints the trapezoidal constraints on translation
     * @param rotationConstraints    the trapezoidal constraints on rotation
     */
    public Pose2dProfile(TrapezoidProfile.Constraints translationConstraints,
            TrapezoidProfile.Constraints rotationConstraints) {
        this.translationProfile = new TrapezoidProfile(translationConstraints);
        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
    }

    /**
     * Create a new Pose2dProfile for motion profiling movement.
     * 
     * @param constraints the constraints on movement
     */
    public Pose2dProfile(PathConstraints constraints) {
        this(new TrapezoidProfile.Constraints(constraints.maxVelocityMPS(), constraints.maxAccelerationMPSSq()),
                new TrapezoidProfile.Constraints(constraints.maxAngularVelocityRadPerSec(),
                        constraints.maxAngularAccelerationRadPerSecSq()));
    }

    /**
     * Calculate a setpoint for the pose of the drivebase.
     * 
     * @param currentPose   the current pose of the drivebase
     * @param chassisSpeeds the field-relative chassis speeds
     * @param targetPose    the target pose
     * @return the next pose setpoint to use
     */
    public Pose2d calculate(Pose2d currentPose, ChassisSpeeds chassisSpeeds, Pose2d targetPose) {
        double velocity = VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .projection(
                        VecBuilder.fill(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY()))
                .norm();
        Translation2d translation = targetPose.minus(currentPose).getTranslation();
        Translation2d finalTranslation = currentPose.getTranslation()
                .plus(translation.times(translationProfile.calculate(0.02, new TrapezoidProfile.State(0, velocity),
                        new TrapezoidProfile.State(translation.getNorm(), 0)).position / translation.getNorm()));
        Rotation2d rotation = targetPose.minus(currentPose).getRotation();
        Rotation2d finalRotation = currentPose.getRotation()
                .plus(Rotation2d.fromRadians(rotationProfile.calculate(0.02,
                        new TrapezoidProfile.State(0, chassisSpeeds.omegaRadiansPerSecond),
                        new TrapezoidProfile.State(rotation.getRadians(), 0)).position));
        return new Pose2d(finalTranslation, finalRotation);
    }
}
