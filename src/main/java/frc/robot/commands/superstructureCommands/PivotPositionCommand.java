// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructureCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class PivotPositionCommand extends Command {
    private final Superstructure pivot;
    private final Supplier<Rotation2d> pivotAngle;

    /**
     * Creates a new PivotCommand that accepts a supplier for a Rotation2d.
     * @param pivot The {@link Superstructure} to control.
     * @param pivotAngle The angle to rotate to, as a supplier for a {@link Rotation2d}. 
     */
    public PivotPositionCommand(Superstructure pivot, Supplier<Rotation2d> pivotAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.pivot = pivot;
        this.pivotAngle = pivotAngle;
        addRequirements(pivot);
    }

     /**
     * Creates a new PivotCommand without a supplier, with input as a Rotation2d.
     * @param pivot The {@link Superstructure} to control.
     * @param pivotAngle The angle to rotate to, as a {@link Rotation2d}
     */
    public PivotPositionCommand(Superstructure pivot, Rotation2d pivotAngle) {
        this(pivot, () -> pivotAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.setRotation(new Rotation2d(MathUtil.clamp(pivotAngle.get().plus(pivot.getPivotRotation()).getDegrees(), PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE)));
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
