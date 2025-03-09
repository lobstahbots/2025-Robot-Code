// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.math;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants.Poses;
import frc.robot.util.trajectory.AlliancePoseMirror;

/**
 * This class stores relevant methods for mathematical operations, conversions,
 * and scaling.
 */
public class LobstahMath {

    /**
     * Scales a number on a range of values to a corresponding value on a different
     * range
     * 
     * @param x         The number to scale.
     * @param inputMin  The original range's lower bound
     * @param inputMax  The original range's upper bound
     * @param outputMin The new range's lower bound
     * @param outputMax The new range's upper bound
     */
    public static double scaleNumberToRange(double x, double inputMin, double inputMax, double outputMin,
            double outputMax) {
        double inputRange = inputMax - inputMin;
        double outputRange = outputMax - outputMin;

        if (inputRange == 0) throw new IllegalArgumentException("Input range cannot be 0");

        return ((x - inputMin) / inputRange * outputRange) + outputMin;

    }

    /**
     * Clamps and scales a number to a range of values to a corresponding value on a
     * different range
     * 
     * @param x         The number to scale.
     * @param inputMin  The original range's lower bound
     * @param inputMax  The original range's upper bound
     * @param outputMin The new range's lower bound
     * @param outputMax The new range's upper bound
     */
    public static double scaleNumberToClampedRange(double x, double inputMin, double inputMax, double outputMin,
            double outputMax) {
        x = MathUtil.clamp(x, inputMin, outputMax);
        return scaleNumberToRange(x, inputMin, inputMax, outputMin, outputMax);
    }

    /**
     * Calculates turning output based on current and desired angle, for gyro values
     * clamped between 180 and -180 degrees.
     * 
     * @param currentAngle The current gyro heading in degrees, 180 to -180.
     * @param desiredAngle The desired gyro heading in degrees, 180 to -180.
     */
    public static double calculateTurningOutput(double currentAngle, double desiredAngle) {
        double output = currentAngle - desiredAngle;
        output %= 360;
        if (Math.abs(output) > 180) output -= Math.signum(output) * 360;
        return output;
    }

    /**
     * Wraps value to fit within a range.
     * 
     * @param lowThreshold  Lowest acceptable value.
     * @param highThreshold Highest acceptable value.
     */
    public static double wrapValue(double value, double lowThreshold, double highThreshold) {
        double range = highThreshold - lowThreshold;

        if (value < lowThreshold) {
            while (value < lowThreshold) {
                value += range;
            }
        }
        if (value > highThreshold) { value %= range; }

        return value;
    }

    /**
     * Unwraps an angle that has been previously wrapped from -pi to pi.
     * 
     * @param ref   The reference angle
     * @param angle The angle to adjust
     * @return The adjusted angle
     */
    public static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    /**
     * Obtains a Rotation2d that points in the opposite direction from this
     * rotation.
     * 
     * @return The rotation rotated by 180 degrees.
     */
    public static Rotation2d flipRotation(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.fromRadians(Math.PI));
    }

    /**
     * Converts a {@link ChassisSpeeds} object to a {@link Twist2d}.
     * 
     * @param chassisSpeeds The ChassisSpeeds to convert
     * @return A Twist2d representing the same vxMetersPerSecond, vyMetersPerSecond,
     *         and omegaRadiansPerSecond.
     */
    public static Twist2d toTwist2d(ChassisSpeeds chassisSpeeds) {
        return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Gets the distance between two {@link Pose2d}s.
     * 
     * @param initialPose The first pose
     * @param endingPose  The second pose
     * @return The distance in meters
     */
    public static double getDistBetweenPoses(Pose2d firstPose, Pose2d secondPose) {
        return firstPose.minus(secondPose).getTranslation().getNorm();
    }

    /**
     * Gets the 2D distance between a {@link Pose3d} and a {@link Pose2d}.
     * 
     * @param firstPose  The first pose
     * @param secondPose The second pose
     * @return The distance in meters
     */
    public static double getDistBetweenPoses(Pose3d firstPose, Pose2d secondPose) {
        return getDistBetweenPoses(firstPose.toPose2d(), secondPose);
    }

    /**
     * Gets the 2D distance between a {@link Pose2d} and a {@link Pose3d}.
     * 
     * @param firstPose  The first pose
     * @param secondPose The second pose
     * @return The distance in meters
     */
    public static double getDistBetweenPoses(Pose2d firstPose, Pose3d secondPose) {
        return getDistBetweenPoses(secondPose, firstPose);
    }

    /**
     * Get the nearest scoring pose from {@link Poses#REEF_POSES} to the current
     * robot pose
     * 
     * @param currentPose the current robot pose, blue alliance origin
     * @param ccw         whether or not to use the counterclockwise branch - that
     *                    is, the right branch for the nearest face of the reef, or
     *                    the left branch for the farthest face
     * @return nearest scoring pose, blue alliance origin
     */
    public static Pose2d getNearestScoringPose(Pose2d currentPose, boolean ccw) {
        var translation = AlliancePoseMirror.mirrorPose2d(currentPose).minus(Poses.REEF_CENTER).getTranslation();
        var angle = wrapValue(translation.getAngle().getRadians() + 7 * Math.PI / 6, 0, 2 * Math.PI);
        Logger.recordOutput("AutoAlignAngle", angle);
        Logger.recordOutput("AutoAlignTranslation", translation);
        return AlliancePoseMirror
                .mirrorPose2d(Poses.REEF_POSES[2 * ((int) angle) + (ccw ? 1 : 0)]);
    }
}
