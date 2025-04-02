// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final PIDController xController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
            DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);
    private static final PIDController yController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
    DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);
    private static final PIDController thetaController = new PIDController(DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private final DriveBase driveBase;
    private final boolean ccw;
    private Pose2d targetPose;

    /** Creates a new DriveToPoseCommand. */
    public AlignToReefCommand(DriveBase driveBase, boolean ccw) {
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        this.ccw = ccw;
        SmartDashboard.putData("X Auto Align PID", xController);
        SmartDashboard.putData("Y Auto Align PID", yController);
    }

    @Override
    public void initialize() {
        // LEDs.getInstance().setAligning(true);
        xController.reset();
        yController.reset();
        thetaController.reset();
        targetPose = LobstahMath.getNearestScoringPose(driveBase.getPose(), ccw);
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
                thetaController.calculate(driveBase.getPose().getRotation().getRadians()),
                driveBase.getPose().getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveBase.stopMotors();
        // LEDs.getInstance().setAligning(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
