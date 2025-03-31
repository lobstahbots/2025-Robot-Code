// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.math.LobstahMath;
import frc.robot.util.trajectory.AlliancePoseMirror;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBargeCommand extends Command {
    /** Creates a new AlignToBargeCommand. */
    private final PIDController xController = new PIDController(DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            0.05, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController yController = new PIDController(DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            0.05, DriveConstants.TRANSLATION_PID_CONSTANTS.kD);
    private final PIDController thetaController = new PIDController(DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private final DriveBase driveBase;
    
    private Pose2d targetPose;

    public AlignToBargeCommand(DriveBase driveBase) {
        // Use addRequirements() here to declare subsystem dependencies.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        addRequirements(driveBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();

        targetPose = AlliancePoseMirror.mirrorPose2d(new Pose2d(FieldConstants.Poses.BARGE_TRANSLATION_DEPTH_SETPOINT, targetPose.getY(), Rotation2d.fromRadians(0)));

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double translationScaling = LobstahMath.getDistBetweenPoses(driveBase.getPose(), targetPose) + 0.25;
        driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationScaling * xController.calculate(driveBase.getPose().getX()), translationScaling * yController.calculate(driveBase.getPose().getY()),
                thetaController.calculate(driveBase.getPose().getRotation().getRadians()),
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
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
