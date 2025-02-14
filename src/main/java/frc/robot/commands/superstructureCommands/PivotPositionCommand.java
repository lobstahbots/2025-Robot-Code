// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructureCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotPositionCommand extends Command {
    private final Superstructure pivot;
    private final DoubleSupplier pivotAngle;


    /** Creates a new PivotCommand. */
    public PivotPositionCommand(Superstructure pivot, DoubleSupplier pivotAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.pivot = pivot;
        this.pivotAngle = pivotAngle;
        addRequirements(pivot);
    }

    public PivotPositionCommand(Superstructure pivot, double pivotAngle) {
        this(pivot, () -> pivotAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.setRotation(new Rotation2d(MathUtil.clamp(pivotAngle.getAsDouble(), PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
