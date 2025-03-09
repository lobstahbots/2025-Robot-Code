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
import frc.robot.util.math.LobstahMath;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
 * command-based.html#defining-commands
 */
public class AlignToReefCommand extends Command {
    private final PIDController xController = new PIDController(DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController yController = new PIDController(DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController thetaController = new PIDController(DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private final DriveBase driveBase;
    private final boolean ccw;

    /** Creates a new DriveToPoseCommand. */
    public AlignToReefCommand(DriveBase driveBase, boolean ccw) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        this.ccw = ccw;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = LobstahMath.getNearestScoringPose(driveBase.getPose(), ccw);
        Logger.recordOutput("AutoAlignTargetPose", targetPose);
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.calculate(driveBase.getPose().getX()), yController.calculate(driveBase.getPose().getY()),
                thetaController.calculate(driveBase.getPose().getRotation().getRadians()), driveBase.getGyroAngle()));
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
