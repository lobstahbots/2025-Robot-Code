package frc.robot.subsystems.endEffector.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEndEffector extends SubsystemBase {
    private final AlgaeEndEffectorIOInputsAutoLogged inputs = new AlgaeEndEffectorIOInputsAutoLogged();
    private final AlgaeEndEffectorIO io;

    public AlgaeEndEffector(AlgaeEndEffectorIO io) {
        this.io = io;
    }

    public void stopMotor() {
        io.stopMotor();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeEndEffector", inputs);
    }

    public void setIdleMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }
}