// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.trajectory.AlliancePoseMirror;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBargeCommand extends Command {
    /** Creates a new AlignToBargeCommand. */
    private final PIDController xController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
            DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);
    
    private final PIDController thetaController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
        DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);

    private final DriveBase driveBase;

    private final DoubleSupplier ySupplier;
    
    private Pose2d targetPose;

    public AlignToBargeCommand(DriveBase driveBase, DoubleSupplier ySupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveBase = driveBase;
        this.ySupplier = ySupplier;
        addRequirements(driveBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.reset();
        thetaController.reset();

        targetPose = AlliancePoseMirror.mirrorPose2d(new Pose2d(FieldConstants.Poses.BARGE_TRANSLATION_DEPTH_SETPOINT, targetPose.getY(), Rotation2d.fromRadians(0)));

        xController.setSetpoint(targetPose.getX());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.calculate(driveBase.getPose().getX()), ySupplier.getAsDouble(),
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
        return xController.atSetpoint() && thetaController.atSetpoint();
    }
}
