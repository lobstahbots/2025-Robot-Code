package frc.robot.subsystems.endEffector.coral;

import java.util.function.DoubleConsumer;

public class CoralEndEffectorIOSim implements CoralEndEffectorIO {
    private double currentSpeed = 0;
    private final DoubleConsumer ejectCallback;
    private boolean hasCoral = true;

    public CoralEndEffectorIOSim(DoubleConsumer callback) {
        ejectCallback = callback;
    }

    public void setIdleMode(boolean isBrake) {}

    public void stopMotor() {
        setSpeed(0);
    }

    public void setVoltage(double voltage) {
        setSpeed(voltage / 12);
    }

    public void setSpeed(double speed) {
        if (currentSpeed > -0.4 && speed <= -0.4 && hasCoral) {
            ejectCallback.accept(speed);
            hasCoral = false;
        }
        if (speed > 0.4) hasCoral = true;
        currentSpeed = speed;
    }

    public void updateInputs(CoralEndEffectorIOInputs inputs) {
        inputs.appliedVoltage = 12 * currentSpeed;
        inputs.beamBreakTriggered = hasCoral;
        inputs.velocity = 3000 * currentSpeed;
        inputs.tempCelsius = 25;
        inputs.currentAmps = currentSpeed != 0 ? 5 : 0;
    }
}
