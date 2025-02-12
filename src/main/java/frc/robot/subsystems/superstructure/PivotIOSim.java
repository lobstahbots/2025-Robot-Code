package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;

public class PivotIOSim implements PivotIO{
    private final DCMotor pivotGearBox = DCMotor.getFalcon500(1);
    private final Mechanism2d pivot = new Mechanism2d(1,1); //TODO: find real values
    private final MechanismRoot2d root = pivot.getRoot("pivot", PivotConstants.X_COORD, Constants.PivotConstants.Y_COORD);
    private final MechanismLigament2d coral;
    private final MechanismLigament2d arm;
    //private final MechanismLigament2d algae;

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        pivotGearBox, 
        PivotConstants.PIVOT_GEARING, 
        SingleJointedArmSim.estimateMOI(PivotConstants.ARM_LENGTH, PivotConstants.PIVOT_MASS), 
        PivotConstants.ARM_LENGTH, 
        PivotConstants.PIVOT_MIN_SIM_ANGLE, 
        PivotConstants.PIVOT_MAX_SIM_ANGLE, 
        true, 
        0, 
        0);

    public PivotIOSim() {
        this.coral = root.append(new MechanismLigament2d("coral", 0, -45)); //TODO figure out what the constants should be same for line 40
        this.coral.setColor(new Color8Bit(Color.kBlue));
        this.arm = root.append(new MechanismLigament2d("arm", 0, -45));
        this.arm.setColor(new Color8Bit(Color.kYellow));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.supplyCurrent = pivotSim.getCurrentDrawAmps();
        inputs.position = pivotSim.getAngleRads();
    }

    @Override
    public void setVoltage(double voltage) {
        pivotSim.setInput(VecBuilder.fill(voltage));
        arm.setAngle(270 - pivotSim.getAngleRads() * 180 / Math.PI);
        pivotSim.update(SimConstants.LOOP_TIME);
    }

    @Override
    public void stop() {
        pivotSim.setInput(0);
    }
}
