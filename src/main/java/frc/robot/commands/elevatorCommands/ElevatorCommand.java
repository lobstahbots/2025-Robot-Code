package frc.robot.commands.elevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier elevatorSpeed;
    public ElevatorCommand(Elevator elevator, DoubleSupplier elevatorSpeed) {
        this.elevator = elevator;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(elevator);
    }
    public ElevatorCommand(Elevator elevator, double elevatorSpeed) {
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
