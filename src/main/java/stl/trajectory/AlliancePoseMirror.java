// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package stl.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import stl.math.LobstahMath;

/** Add your docs here. */
public class AlliancePoseMirror {

  /**
   * Returns mirrored X coordinate depending on alliance color.
   * @param xPos The X coordinate to mirror
   * @return A corrected X coordinate (mirrored or unmirrored) depending on alliance color.
   */
  public static double mirrorX(double xPos) {
    if (isRedAlliance()) {
      return FieldConstants.FIELD_LENGTH - xPos;
    } else {
      return xPos;
    }
  }

  /**
   * Returns mirrored Translation2d depending on alliance color.
   * @param translation The Translation2d to mirror
   * @return A corrected Translation2d (mirrored or unmirrored) depending on alliance color.
   */
  public static Translation2d mirrorTranslation2d(Translation2d translation) {
    if (isRedAlliance()) {
      return new Translation2d(mirrorX(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Returns mirrored Translation3d depending on alliance color.
   * @param translation The Translation3d to mirror
   * @return A corrected Translation3d (mirrored or unmirrored) depending on alliance color.
   */
  public static Translation3d mirrorTranslation3d(Translation3d translation) {
    if (isRedAlliance()) {
      return new Translation3d(mirrorX(translation.getX()), translation.getY(), translation.getZ());
    } else {
      return translation;
    }
  }

  /**
   * Returns mirrored Rotation2d depending on alliance color.
   * @param rotation The Rotation2d to mirror
   * @return A corrected Rotation2d (mirrored or unmirrored) depending on alliance color.
   */
  public static Rotation2d mirrorRotation2d(Rotation2d rotation) {
    if (isRedAlliance()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

   /**
   * Returns mirrored Rotation3d depending on alliance color.
   * @param rotation The Rotation3d to mirror
   * @return A corrected Rotation3d (mirrored or unmirrored) depending on alliance color.
   */
  public static Rotation3d mirrorRotation3d(Rotation3d rotation) {
    if (isRedAlliance()) {
      return new Rotation3d(rotation.getX(), rotation.getY(), mirrorRotation2d(new Rotation2d(rotation.getZ())).getRadians());
    } else {
      return rotation;
    }
  }

  /**
   * Returns mirrored Pose2d depending on alliance color.
   * @param pose The Pose2d to mirror
   * @return A corrected Pose2d (mirrored or unmirrored) depending on alliance color.
   */
  public static Pose2d mirrorPose2d(Pose2d pose) {
    if (isRedAlliance()) {
      return new Pose2d(mirrorTranslation2d(pose.getTranslation()), mirrorRotation2d(pose.getRotation()));
    } else {
      return pose;
    }
  }

  /**
   * Returns mirrored Pose3d depending on alliance color.
   * @param pose The Pose3d to mirror
   * @return A corrected Pose3d (mirrored or unmirrored) depending on alliance color.
   */
  public static Pose3d mirrorPose3d(Pose3d pose) {
    if (isRedAlliance()) {
      return new Pose3d(mirrorTranslation3d(pose.getTranslation()), mirrorRotation3d(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Rotation2d flipRotation(Rotation2d rotation) {
    if (isRedAlliance()) {
        return LobstahMath.flipRotation(rotation);
    } 
    return rotation;
  }

  /**
   * @return Whether the robot is on the red alliance side and positions should be mirrored.
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
