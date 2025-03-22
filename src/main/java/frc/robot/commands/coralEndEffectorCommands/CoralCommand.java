// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralEndEffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.coral.CoralEndEffector;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
 * command-based.html#defining-commands
 */
public class CoralCommand extends Command {
    /** Creates a new OuttakeCommand. */
    private final CoralEndEffector endEffector;
    private final double speed;

    public CoralCommand(CoralEndEffector endEffector, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.endEffector = endEffector;
        this.speed = speed;
        addRequirements(endEffector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        endEffector.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        endEffector.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return endEffector.getCurrent() > 40;
        return false;
    }
}
