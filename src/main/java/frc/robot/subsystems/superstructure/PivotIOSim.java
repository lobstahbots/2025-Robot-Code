package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.SimShared;

public class PivotIOSim implements PivotIO{
    private final DCMotor pivotGearBox = DCMotor.getFalcon500(1);

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        pivotGearBox, 
        PivotConstants.PIVOT_GEARING, 
        SingleJointedArmSim.estimateMOI(PivotConstants.ARM_LENGTH, PivotConstants.PIVOT_MASS), 
        PivotConstants.ARM_LENGTH, 
        PivotConstants.MIN_ANGLE.getRadians(), 
        PivotConstants.MAX_ANGLE.getRadians(), 
        true, 
        0);

    public PivotIOSim() {}

    public void updateInputs(PivotIOInputs inputs) {
        inputs.supplyCurrent = pivotSim.getCurrentDrawAmps();
        inputs.position = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.velocity = pivotSim.getVelocityRadPerSec();
        inputs.temperature = 25;
        inputs.appliedVoltage = pivotSim.getInput(0);
        inputs.torqueCurrent = inputs.supplyCurrent;
        inputs.statorCurrent = inputs.supplyCurrent;

        pivotSim.update(SimConstants.LOOP_TIME);

        SimShared.powerDistributionSim.setCurrent(SimConstants.PIVOT_CHANNEL, inputs.supplyCurrent);
    }

    public void setVoltage(double voltage) {
        pivotSim.setInputVoltage(voltage);
    }

    public void stop() {
        pivotSim.setInputVoltage(0);
    }

    @Override
    public void setIdleMode(boolean isBrake) { }
}
