// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructureCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
    private final Superstructure pivot;
    private final DoubleSupplier pivotSpeed;


    /** Creates a new PivotCommand. */
    public PivotCommand(Superstructure pivot, DoubleSupplier pivotSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.pivot = pivot;
        this.pivotSpeed = pivotSpeed;
        addRequirements(pivot);
    }

    public PivotCommand(Superstructure pivot, double pivotSpeed) {
        this(pivot, () -> pivotSpeed);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.setPivotVoltage(pivotSpeed.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivot.setPivotVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
