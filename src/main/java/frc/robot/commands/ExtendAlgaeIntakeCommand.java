package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;

public class ExtendAlgaeIntakeCommand extends Command {
    private final AlgaeIntake algaeIntake;
    private final double speed;
    
    public ExtendAlgaeIntakeCommand(AlgaeIntake algaeIntake, double speed) {
        this.algaeIntake = algaeIntake;
        this.speed = speed;
        addRequirements(algaeIntake);
    }

    @Override
    public void initialize() {
        algaeIntake.setIntakePivotSpeed(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted){
        algaeIntake.stopIntakePivot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

