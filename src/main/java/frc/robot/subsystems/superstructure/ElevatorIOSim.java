package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.SimShared;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox = DCMotor.getFalcon500(2);
    private final ElevatorSim elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_MASS, ElevatorConstants.PITCH_DIAMETER, ElevatorConstants.BOTTOM_HEIGHT,
            ElevatorConstants.TOP_HEIGHT, true, ElevatorConstants.BOTTOM_HEIGHT);

    private final TalonFX leftMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_ID);
    private final TalonFX rightMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_ID);

    private final TalonFXSimState leftMotorSim = leftMotor.getSimState();
    private final TalonFXSimState rightMotorSim = rightMotor.getSimState();

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightAppliedVoltage;
    private final StatusSignal<Current> rightSupplyCurrent;
    private final StatusSignal<Current> rightStatorCurrent;
    private final StatusSignal<Current> rightTorqueCurrent;
    private final StatusSignal<Temperature> rightTempCelsius;
    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftAppliedVoltage;
    private final StatusSignal<Current> leftSupplyCurrent;
    private final StatusSignal<Current> leftStatorCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;
    private final StatusSignal<Temperature> leftTempCelsius;
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(
            ElevatorConstants.MOTION_MAGIC_POSITION_VOLTAGE);
    private final VoltageOut voltageOut = new VoltageOut(ElevatorConstants.VOLTAGE_OUTPUT).withEnableFOC(false);

    public ElevatorIOSim() {
        leftMotorSim.Orientation = ChassisReference.Clockwise_Positive;
        rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER
                * Math.PI;

        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kS = ElevatorConstants.kS;
        config.Slot0.kV = ElevatorConstants.kV;
        config.Slot0.kA = ElevatorConstants.kA;
        config.Slot0.kG = ElevatorConstants.kG;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightAppliedVoltage = rightMotor.getMotorVoltage();
        rightSupplyCurrent = rightMotor.getSupplyCurrent();
        rightStatorCurrent = rightMotor.getStatorCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTempCelsius = rightMotor.getDeviceTemp();
        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftAppliedVoltage = leftMotor.getMotorVoltage();
        leftSupplyCurrent = leftMotor.getSupplyCurrent();
        leftStatorCurrent = leftMotor.getStatorCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTempCelsius = leftMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.BASE_STATUS_SIGNAL_FREQUENCY, rightPosition,
                rightVelocity, rightAppliedVoltage, rightSupplyCurrent, rightTorqueCurrent, rightTempCelsius,
                leftPosition, leftVelocity, leftAppliedVoltage, leftSupplyCurrent, leftTorqueCurrent, leftTempCelsius);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(rightMotorSim.getMotorVoltage());
        elevatorSim.update(SimConstants.LOOP_TIME);
        rightMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters()
                / (ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI));
        leftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters()
                / (ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI));
        rightMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()
                / (ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI));
        leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()
                / (ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI));

        BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVoltage, rightSupplyCurrent,
                rightStatorCurrent, rightTorqueCurrent, rightTempCelsius, leftPosition, leftVelocity,
                leftAppliedVoltage, leftSupplyCurrent, leftStatorCurrent, leftTorqueCurrent, leftTempCelsius);

        inputs.rightPosition = rightPosition.getValueAsDouble();
        inputs.rightVelocity = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVoltage = rightAppliedVoltage.getValueAsDouble();
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValueAsDouble();
        inputs.rightStatorCurrent = rightStatorCurrent.getValueAsDouble();
        inputs.rightTorqueCurrent = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
        inputs.leftPosition = leftPosition.getValueAsDouble();
        inputs.leftVelocity = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVoltage = leftAppliedVoltage.getValueAsDouble();
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValueAsDouble();
        inputs.leftStatorCurrent = leftStatorCurrent.getValueAsDouble();
        inputs.leftTorqueCurrent = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();
        inputs.limitSwitchHit = elevatorSim.hasHitLowerLimit();
        inputs.atSetpoint = MathUtil.applyDeadband(rightMotor.getClosedLoopError().getValueAsDouble(), ElevatorConstants.HEIGHT_DEADBAND) == 0;

        SimShared.powerDistributionSim.setCurrent(SimConstants.ELEVATOR_CHANNELS[0], inputs.leftSupplyCurrent);
        SimShared.powerDistributionSim.setCurrent(SimConstants.ELEVATOR_CHANNELS[1], inputs.rightSupplyCurrent);
    }

    @Override
    public void setPosition(TrapezoidProfile.State state) {
        rightMotor.setControl(positionVoltage.withPosition(state.position));
    }

    @Override
    public void setVoltage(double voltage) {
        rightMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void resetEncoder(double position) {
        elevatorSim.setState(position, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
    }
}
