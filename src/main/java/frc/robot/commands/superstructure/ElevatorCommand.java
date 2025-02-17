package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class ElevatorCommand extends Command {
    private final Superstructure elevator;
    private final DoubleSupplier elevatorSpeed;
    public ElevatorCommand(Superstructure elevator, DoubleSupplier elevatorSpeed) {
        this.elevator = elevator;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(elevator);
    }
    public ElevatorCommand(Superstructure elevator, double elevatorSpeed) {
        this(elevator, ()->elevatorSpeed);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setVoltage(elevatorSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
