// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveBase;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
 * command-based.html#defining-commands
 */
public class DriveToPoseCommand extends Command {
    private final PIDController xController = new PIDController(3 * DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController yController = new PIDController(3 * DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController thetaController = new PIDController(2 * DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private final DriveBase driveBase;

    /** Creates a new DriveToPoseCommand. */
    public DriveToPoseCommand(DriveBase driveBase, Pose2d pose) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        thetaController.setSetpoint(pose.getRotation().getRadians());
        Logger.recordOutput("DriveToPosePose", pose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveBase.driveRobotRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(10 * xController.calculate(driveBase.getPose().getX()),
                        10 * yController.calculate(driveBase.getPose().getY()),
                        10 * thetaController.calculate(driveBase.getPose().getRotation().getRadians()),
                        driveBase.getGyroAngle()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveBase.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
