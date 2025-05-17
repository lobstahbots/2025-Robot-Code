package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    public void zeroGyro() {
        gyroSimulation.setRotation(new Rotation2d());
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.isCalibrating = false;
    }
}
