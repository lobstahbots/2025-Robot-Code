package frc.robot.subsystems.endEffector.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
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

    /**
     * Construct a command to spin the end effector
     * 
     * @param speed speed to spin the motor at
     * @return constructed command
     */
    public Command spinCommand(double speed) {
        return runEnd(() -> io.setSpeed(speed), io::stopMotor);
    }

    /**
     * Construct a command to stop the end effector
     * 
     * @return constructed command
     */
    public Command stopCommand() {
        return run(io::stopMotor);
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralEndEffector", inputs);
    }
}
