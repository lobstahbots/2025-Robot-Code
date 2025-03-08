package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

/**
 * Superstructure move to setpoint command that sets goal. Does not do danger
 * zone avoidance.
 */
public class SuperstructureStateCommand extends Command {
    private final Superstructure superstructure;
    private final Supplier<SuperstructureState> goalSupplier;

    public SuperstructureStateCommand(Superstructure superstructure, Supplier<SuperstructureState> goalSupplier) {
        this.superstructure = superstructure;
        this.goalSupplier = goalSupplier;
        addRequirements(superstructure);
    }

    public SuperstructureStateCommand(Superstructure superstructure, SuperstructureState goal) {
        this(superstructure, () -> goal);
    }

    @Override
    public void initialize() {
        superstructure.reset(superstructure.getState());
    }

    @Override
    public void execute() {
        superstructure.setState(goalSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return superstructure.atPivotSetpoint() && superstructure.atElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // superstructure.stopMotion();
    }
}