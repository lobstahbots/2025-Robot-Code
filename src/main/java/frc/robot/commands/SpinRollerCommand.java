package frc.robot.commands;

import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinRollerCommand extends Command{
    private final AlgaeIntake algaeIntake;
    private final double speed;
    
    public SpinRoller(AlgaeIntake algaeIntake, double speed) {
        this.algaeIntake = algaeIntake;
        this.speed = speed;
        addRequirements(algaeIntake);
    }

    @Override
    public void initialize() {
        algaeIntake.setIntakeRollerSpeed(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted){
        algaeIntake.stopIntakeRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


