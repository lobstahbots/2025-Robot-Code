package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

/** Superstructure move to setpoint command that sets goal. Does not do danger zone avoidance. */
public class SuperstructureStateCommand extends Command {
  private final Superstructure superstructure;
    private final SuperstructureState goal;

  public SuperstructureStateCommand(
      Superstructure superstructure, SuperstructureState goal) {
    this.superstructure = superstructure;
    this.goal = goal;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.reset(superstructure.getState());
  }

  @Override
  public void execute() {
    superstructure.setState(goal);
  }

  @Override
  public boolean isFinished() {
    return superstructure.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopMotion();
  }
}