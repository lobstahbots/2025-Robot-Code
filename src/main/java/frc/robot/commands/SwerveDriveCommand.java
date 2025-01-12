// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.drive.DriveBase;

/** A command to drive a swerve robot. */
public class SwerveDriveCommand extends Command {
  private final DriveBase driveBase;
  private final DoubleSupplier strafeXSupplier;
  private final DoubleSupplier strafeYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldCentric;
  private final boolean squareInputs;

  /**
   * Create a new command to drive a serve robot.
   * 
   * @param driveBase        The {@link DriveBase} to drive.
   * @param strafeXSupplier  Supplier for strafe in X direction, e.g. from a
   *                         joystick.
   * @param strafeYSupplier  Supplier for strafe in Y direction, e.g. from a
   *                         joystick.
   * @param rotationSupplier Supplier for rotation, e.g. from a joystick.
   * @param fieldCentric     Supplier for whether or not to drive field centric.
   * @param squareInputs     If true, square the linear magnitude of the strafe
   *                         suppliers combined and that of the rotation, but
   *                         keeps direction the same.
   */
  public SwerveDriveCommand(DriveBase driveBase, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier,
      DoubleSupplier rotationSupplier, BooleanSupplier fieldCentric, boolean squareInputs) {
    this.driveBase = driveBase;
    this.strafeXSupplier = strafeXSupplier;
    this.strafeYSupplier = strafeYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldCentric = fieldCentric;
    this.squareInputs = squareInputs;
    addRequirements(driveBase);
  }

  /**
   * Create a new command to drive a serve robot.
   * 
   * @param driveBase        The {@link DriveBase} to drive.
   * @param strafeXSupplier  Constant strafe in X direction, e.g. from a joystick.
   * @param strafeYSupplier  Constant strafe in Y direction, e.g. from a joystick.
   * @param rotationSupplier Constant rotation, e.g. from a joystick.
   * @param fieldCentric     Whether or not to drive field centric.
   * @param squareInputs     If true, square the linear magnitude of the strafe
   *                         suppliers combined and that of the rotation, but
   *                         keeps direction the same.
   */
  public SwerveDriveCommand(DriveBase driveBase, double strafeX, double strafeY, double rotation, boolean fieldCentric,
      boolean squareInputs) {
    this(driveBase, () -> strafeX, () -> strafeY, () -> rotation, () -> fieldCentric, squareInputs);
  }

  @Override
  public void execute() {
    if (fieldCentric.getAsBoolean()) {
      double linearMagnitude = MathUtil.applyDeadband(
          Math.hypot(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble()), IOConstants.JOYSTICK_DEADBAND);
      Rotation2d linearDirection = new Rotation2d(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble());
      double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), IOConstants.JOYSTICK_DEADBAND);

      // Square values
      if (squareInputs) {
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);
      }

      // Calculate new linear velocity
      Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.MAX_DRIVE_SPEED,
          linearVelocity.getY() * DriveConstants.MAX_DRIVE_SPEED, omega * DriveConstants.MAX_ANGULAR_SPEED);

      driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, driveBase.getGyroAngle()));
    } else {
      driveBase.driveRobotRelative(new ChassisSpeeds(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble(),
          rotationSupplier.getAsDouble()));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopMotors();
  }
}
