package frc.robot.subsystems.endEffector.algae;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEndEffector extends SubsystemBase {
    private final AlgaeEndEffectorIOInputsAutoLogged inputs = new AlgaeEndEffectorIOInputsAutoLogged();
    private final AlgaeEndEffectorIO io;

    public AlgaeEndEffector(AlgaeEndEffectorIO io) {
        this.io = io;
    }

    private void stopMotor() {
        io.stopMotor();
    }

    /**
     * Construct a command which stops this end effector's motion and does not end.
     * 
     * @return the constructed command
     */
    public Command stop() {
        return run(this::stopMotor);
    }

    /**
     * Constructs a command which spins this end effector at the speed supplied by
     * the double supplier and does not end.
     * 
     * @param speed the supplier which supplies the speed to spin at, should supply
     *              values between -1 and 1
     * @return the constructed command
     */
    public Command spin(DoubleSupplier speed) {
        return run(() -> setSpeed(speed.getAsDouble()));
    }

    /**
     * Constructs a command which spins this end effector at the speed specified and
     * does not end.
     * 
     * @param speed the speed to spin at, between -1 and 1
     * @return the constructed command
     */
    public Command spin(double speed) {
        return run(() -> setSpeed(speed));
    }
    private void setSpeed(double speed) {
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