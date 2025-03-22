// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.profile.Pose2dProfile;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.math.LobstahMath;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
 * command-based.html#defining-commands
 */
public class AlignToReefCommand extends Command {
    private final PIDController xController = new PIDController(1.5 * DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController yController = new PIDController(DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController thetaController = new PIDController(DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private Pose2dProfile profile = new Pose2dProfile(PathConstants.CONSTRAINTS);

    private final DriveBase driveBase;
    private final boolean ccw;
    private Pose2d targetPose;

    /** Creates a new DriveToPoseCommand. */
    public AlignToReefCommand(DriveBase driveBase, boolean ccw) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        this.ccw = ccw;
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
        targetPose = LobstahMath.getNearestScoringPose(driveBase.getPose(), ccw);
        Logger.recordOutput("AutoAlignTargetPose", targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d setpoint = profile.calculate(driveBase.getPose(),
                driveBase.getFieldRelativeChassisSpeeds(driveBase.getRobotRelativeSpeeds()), targetPose);
        driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.calculate(driveBase.getPose().getX(), setpoint.getX()),
                yController.calculate(driveBase.getPose().getY(), setpoint.getY()), thetaController
                        .calculate(driveBase.getPose().getRotation().getRadians(), setpoint.getRotation().getRadians()),
                driveBase.getPose().getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveBase.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Transform2d diff = targetPose.minus(driveBase.getPose());
        return Math.abs(diff.getX()) < 0.01 && Math.abs(diff.getY()) < 0.01
                && Math.abs(diff.getRotation().getRadians()) < 0.05;
    }
}
