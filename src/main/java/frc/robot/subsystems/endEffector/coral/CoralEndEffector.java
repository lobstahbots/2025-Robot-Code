package frc.robot.subsystems.endEffector.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralEndEffector extends SubsystemBase {
    private final CoralEndEffectorIOInputsAutoLogged inputs = new CoralEndEffectorIOInputsAutoLogged();
    private final CoralEndEffectorIO io;

    public CoralEndEffector(CoralEndEffectorIO io) {
        this.io = io;
    }

    public void stopMotor() {
        io.stopMotor();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralEndEffector", inputs);
    }

    public void setIdleMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }
}
