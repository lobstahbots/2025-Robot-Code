package frc.robot.util.sysId;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class CharacterizableSubsystem extends SubsystemBase {
    public CharacterizableSubsystem() {
        super();
    }

    public abstract void runVolts(double volts);
}