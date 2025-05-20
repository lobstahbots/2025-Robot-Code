package frc.robot.subsystems.endEffector.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.led.LEDs;

public class CoralEndEffector extends SubsystemBase {
    private final CoralEndEffectorIOInputsAutoLogged inputs = new CoralEndEffectorIOInputsAutoLogged();
    private final CoralEndEffectorIO io;
    public final Trigger beamBreak = new Trigger(() -> inputs.beamBreakTriggered).debounce(0.5);

    public CoralEndEffector(CoralEndEffectorIO io) {
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

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    public boolean getBeamBreak() {
        return inputs.beamBreakTriggered;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralEndEffector", inputs);
        LEDs.getInstance().setHasCoral(inputs.beamBreakTriggered);
    }

    public void setIdleMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }
}
