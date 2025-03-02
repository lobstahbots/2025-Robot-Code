// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class ElevatorPositionCommand extends Command {
  private final Superstructure superstructure;
  private final DoubleSupplier elevatorPosition;
  /** Creates a new ElevatorPositionCommand. */
  public ElevatorPositionCommand(Superstructure superstructure, DoubleSupplier elevatorPosition) {
    this.superstructure = superstructure;
    this.elevatorPosition = elevatorPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);
  }
  public ElevatorPositionCommand(Superstructure superstructure, double elevatorPosition) {
    this(superstructure, () -> elevatorPosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    superstructure.setExtension(elevatorPosition.getAsDouble(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return superstructure.atElevatorSetpoint();
  }
}
