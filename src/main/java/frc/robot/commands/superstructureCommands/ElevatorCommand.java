package frc.robot.commands.superstructureCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class ElevatorCommand extends Command {
    private final Superstructure superstructure;
    private final DoubleSupplier elevatorSpeed;
    public ElevatorCommand(Superstructure superstructure, DoubleSupplier elevatorSpeed) {
        this.superstructure = superstructure;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(superstructure);
    }
    public ElevatorCommand(Superstructure superstructure, double elevatorSpeed) {
        this(superstructure, () -> elevatorSpeed);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        superstructure.setElevatorVoltage(elevatorSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setElevatorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
