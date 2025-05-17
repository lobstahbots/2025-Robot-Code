package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.SimShared;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox = DCMotor.getFalcon500(2);
    private final ElevatorSim elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_MASS, ElevatorConstants.PITCH_DIAMETER, ElevatorConstants.BOTTOM_HEIGHT,
            ElevatorConstants.TOP_HEIGHT, true, ElevatorConstants.BOTTOM_HEIGHT);

    public ElevatorIOSim() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(SimConstants.LOOP_TIME);
        
        inputs.leftPosition = inputs.rightPosition = elevatorSim.getPositionMeters()
                * ElevatorConstants.ELEVATOR_SIM_RATIO;
        inputs.leftVelocity = inputs.rightVelocity = elevatorSim.getPositionMeters()
                * ElevatorConstants.ELEVATOR_SIM_RATIO;
        inputs.leftAppliedVoltage = inputs.rightAppliedVoltage = elevatorSim.getInput(0);
        inputs.leftSupplyCurrent = inputs.rightSupplyCurrent = elevatorSim.getCurrentDrawAmps();
        inputs.leftStatorCurrent = inputs.rightStatorCurrent = inputs.rightSupplyCurrent;
        inputs.rightStatorCurrent = inputs.rightTorqueCurrent = inputs.rightSupplyCurrent;
        inputs.leftTempCelsius = inputs.rightTempCelsius = 25;
        inputs.limitSwitchHit = elevatorSim.hasHitLowerLimit();

        SimShared.powerDistributionSim.setCurrent(SimConstants.ELEVATOR_CHANNELS[0], inputs.leftSupplyCurrent);
        SimShared.powerDistributionSim.setCurrent(SimConstants.ELEVATOR_CHANNELS[1], inputs.rightSupplyCurrent);
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void resetEncoder(double position) {
        elevatorSim.setState(position, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void stop() {
        elevatorSim.setInputVoltage(0);
    }

    @Override
    public void setIdleMode(boolean isBrake) {}
}
