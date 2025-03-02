// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class PivotPositionCommand extends Command {
    private final Superstructure superstructure;
    private final Supplier<Rotation2d> pivotAngle;

    /**
     * Creates a new PivotCommand that accepts a supplier for a Rotation2d.
     * @param superstructure The {@link Superstructure} to control.
     * @param pivotAngle The angle to rotate to, as a supplier for a {@link Rotation2d}. 
     */
    public PivotPositionCommand(Superstructure superstructure, Supplier<Rotation2d> pivotAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.superstructure = superstructure;
        this.pivotAngle = pivotAngle;
        addRequirements(superstructure);
    }

     /**
     * Creates a new PivotCommand without a supplier, with input as a Rotation2d.
     * @param superstructure The {@link Superstructure} to control.
     * @param pivotAngle The angle to rotate to, as a {@link Rotation2d}
     */
    public PivotPositionCommand(Superstructure superstructure, Rotation2d pivotAngle) {
        this(superstructure, () -> pivotAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        superstructure.setRotation(Rotation2d.fromRotations(MathUtil.clamp(pivotAngle.get().getRadians(), PivotConstants.MIN_ANGLE.getRadians(), PivotConstants.MAX_ANGLE.getRadians())));
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
