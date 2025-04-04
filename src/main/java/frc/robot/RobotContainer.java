// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.AutoFactory.CoralStation;
import frc.robot.AutoFactory.StartingPosition;
import frc.robot.Constants.AlgaeEndEffectorConstants;
import frc.robot.Constants.CoralEndEffectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Poses;
import frc.robot.Constants.IOConstants.ControllerIOConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.algaeEndEffector.AlgaeCommand;
import frc.robot.commands.algaeEndEffector.StopAlgaeCommand;
import frc.robot.commands.coralEndEffectorCommands.CoralCommand;
import frc.robot.commands.drivebase.AlignToBargeCommand;
import frc.robot.commands.drivebase.AlignToProcessorCommand;
import frc.robot.commands.drivebase.AlignToReefCommand;
import frc.robot.commands.drivebase.SwerveDriveCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.endEffector.algae.AlgaeEndEffector;
import frc.robot.subsystems.endEffector.algae.AlgaeEndEffectorIOSparkMax;
import frc.robot.subsystems.endEffector.coral.CoralEndEffector;
import frc.robot.subsystems.endEffector.coral.CoralEndEffectorIOSparkMax;
import frc.robot.subsystems.superstructure.ElevatorIOSim;
import frc.robot.subsystems.superstructure.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.PivotIOSim;
import frc.robot.subsystems.superstructure.PivotIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.util.auto.AutonSelector;
import frc.robot.util.auto.AutonSelector.AutoQuestion;
import frc.robot.util.led.LEDs;
import frc.robot.util.trajectory.AlliancePoseMirror;

public class RobotContainer {
    private final LEDs leds;

    private final DriveBase driveBase;
    private final Superstructure superstructure;
    private final CoralEndEffector coral;
    private double coralSpeed = -0.75;
    private final AlgaeEndEffector algae;

    //sticks
    private final Joystick driverJoystick = new Joystick(ControllerIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(ControllerIOConstants.OPERATOR_CONTROLLER_PORT);

    //Driver
    private final Trigger driverLTButton = new Trigger(
            () -> driverJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.2); //Outake Coral
    private final Trigger driverRTButton = new Trigger(
            () -> driverJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.2); //Outake Algae

    private final JoystickButton driverLBButton = new JoystickButton(driverJoystick, ControllerIOConstants.LB_BUTTON); //Barge automation
    private final JoystickButton driverRBButton = new JoystickButton(driverJoystick, ControllerIOConstants.RB_BUTTON); //Intake Sequence

    private final JoystickButton driverLeftPaddle = new JoystickButton(driverJoystick,
            ControllerIOConstants.LEFT_PADDLE); //Autoalign left reef
    private final JoystickButton driverRightPaddle = new JoystickButton(driverJoystick,
            ControllerIOConstants.RIGHT_PADDLE); //Autoalign right reef

    //Operator
    private final Trigger operatorLTButton = new Trigger(
            () -> operatorJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5); //Outake Coral
    private final Trigger operatorRTButton = new Trigger(
            () -> operatorJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5); //Intake Coral

    private final JoystickButton operatorLBButton = new JoystickButton(operatorJoystick,
            ControllerIOConstants.LB_BUTTON); //Outake Algae
    private final JoystickButton operatorRBButton = new JoystickButton(operatorJoystick,
            ControllerIOConstants.RB_BUTTON); //L1 Setpoint (potentially to change)

    private final JoystickButton operatorXButton = new JoystickButton(operatorJoystick, ControllerIOConstants.X_BUTTON); //L2
    private final JoystickButton operatorYButton = new JoystickButton(operatorJoystick, ControllerIOConstants.Y_BUTTON); //L3
    private final JoystickButton operatorBButton = new JoystickButton(operatorJoystick, ControllerIOConstants.B_BUTTON); //L4
    private final JoystickButton operatorAButton = new JoystickButton(operatorJoystick, ControllerIOConstants.A_BUTTON); //Theoretically L1 or intake

    private final JoystickButton operatorLeftPaddle = new JoystickButton(operatorJoystick,
            ControllerIOConstants.LEFT_PADDLE);
    private final JoystickButton operatorRightPaddle = new JoystickButton(operatorJoystick,
            ControllerIOConstants.RIGHT_PADDLE);

    private final POVButton operatorDpadUp = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_UP); // L3 Algae Removal
    private final POVButton operatorDpadDown = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_DOWN); // L2 Algae Removal
    private final POVButton operatorDpadLeft = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_LEFT); // Barge Setpoint
    private final POVButton operatorDpadRight = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_RIGHT); // Processor Setpoint

    private final Trigger manualArm = new Trigger(
            () -> operatorJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL) > 0.1);

    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;

    private SwerveDriveSimulation driveSimulation = null;

    //private int scoreLevel = 1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        leds = new LEDs();
        if (Robot.isReal()) {
            SwerveModuleIOSparkMax frontLeft = new SwerveModuleIOSparkMax(FrontLeftModuleConstants.moduleID,
                    "Front left ", FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax frontRight = new SwerveModuleIOSparkMax(FrontRightModuleConstants.moduleID,
                    " Front right", FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
            SwerveModuleIOSparkMax backLeft = new SwerveModuleIOSparkMax(BackLeftModuleConstants.moduleID, " Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax backRight = new SwerveModuleIOSparkMax(BackRightModuleConstants.moduleID,
                    "Back right", BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);

            List<Camera> cameras = VisionConstants.CAMERA_TRANSFORMS.keySet().stream()
                    .map(name -> new Camera(new CameraIOPhoton(name))).toList();
            driveBase = new DriveBase(new GyroIONavX(), cameras, frontLeft, frontRight, backLeft, backRight, false);

            superstructure = new Superstructure(
                    new ElevatorIOTalonFX(ElevatorConstants.LEFT_ELEVATOR_ID, ElevatorConstants.RIGHT_ELEVATOR_ID),
                    new PivotIOTalonFX(PivotConstants.MOTOR_ID, PivotConstants.ENCODER_ID));

        } else {
            driveSimulation = new SwerveDriveSimulation(DriveConstants.MAPLE_SIM_CONFIG,
                    new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

            var modules = driveSimulation.getModules();
            SwerveModuleIOSim frontLeft = new SwerveModuleIOSim(FrontLeftModuleConstants.angleOffset, modules[0], 0);
            SwerveModuleIOSim frontRight = new SwerveModuleIOSim(FrontRightModuleConstants.angleOffset, modules[1], 1);
            SwerveModuleIOSim backLeft = new SwerveModuleIOSim(BackLeftModuleConstants.angleOffset, modules[2], 2);
            SwerveModuleIOSim backRight = new SwerveModuleIOSim(BackRightModuleConstants.angleOffset, modules[3], 3);

            List<Camera> cameras;
            if (SimConstants.VISION_SIM) {
                cameras = VisionConstants.CAMERA_TRANSFORMS.keySet().stream()
                        .map(name -> new Camera(new CameraIOSim(name))).toList();
            } else {
                cameras = new ArrayList<>();
            }
            driveBase = new DriveBase(new GyroIOSim(driveSimulation.getGyroSimulation()) {}, cameras, frontLeft,
                    frontRight, backLeft, backRight, false);

            superstructure = new Superstructure(new ElevatorIOSim(), new PivotIOSim());
        }

        coral = new CoralEndEffector(new CoralEndEffectorIOSparkMax(CoralEndEffectorConstants.LEFT_ID,
                CoralEndEffectorConstants.BEAM_BREAK_ID));
        algae = new AlgaeEndEffector(new AlgaeEndEffectorIOSparkMax(AlgaeEndEffectorConstants.MOTOR_ID));

        this.autoFactory = new AutoFactory(driveBase, coral, superstructure, autoChooser::getResponses);

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();

    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(new SwerveDriveCommand(driveBase,
                () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL),
                () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL),
                () -> -driverJoystick.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL),
                () -> DriveConstants.FIELD_CENTRIC, ControllerIOConstants.SQUARE_INPUTS,
                () -> 0 / RobotConstants.L4_STATE.elevatorHeight * superstructure.getExtension() + 1));

        SmartDashboard.putData("thing", superstructure);
        coral.setDefaultCommand(new CoralCommand(coral, 0.1));
        algae.setDefaultCommand(new AlgaeCommand(algae, -0.1));
        superstructure
                .setDefaultCommand(
                        Commands.run(
                                () -> superstructure.setState(new SuperstructureState(
                                        superstructure.getGoal().pivotRotation
                                                .minus(Rotation2d.fromRadians(0.1 * operatorJoystick
                                                        .getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL))),
                                        superstructure.getGoal().elevatorHeight + operatorJoystick
                                                .getRawAxis(ControllerIOConstants.RIGHT_STICK_VERTICAL) * -2,
                                        0, 0)),
                                superstructure));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getCommand();
    }

    public void configureButtonBindings() {

        //driver
        driverLTButton.whileTrue(new CoralCommand(coral, () -> coralSpeed));
        // driverRBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE));
        // driverRBButton.whileTrue(new CoralCommand(coral, 0.5));
        // driverRBButton.onTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE)
        //         .alongWith(new CoralCommand(coral, 0.5)).until(() -> coral.getBeamBreak()));

        driverLBButton.whileTrue(Commands.select(Map.of(1,
                new AlignToBargeCommand(driveBase,
                        () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL))
                                .alongWith(superstructure.getSetpointCommand(RobotConstants.BARGE_STATE)),
                2, new AlignToProcessorCommand(driveBase)), () -> {
                    if (AlliancePoseMirror.mirrorPose2d(driveBase.getPose()).getY() > FieldConstants.FIELD_WIDTH / 2)
                        return 1;
                    return 2;
                })); //TEST THIS VERSION FIRST

        driverRBButton.onTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE));
        driverRBButton.whileTrue(new CoralCommand(coral, 0.5));
        driverRBButton.whileTrue(new InstantCommand(() -> leds.setUserSignal(true)).ignoringDisable(true))
                .onFalse(new InstantCommand(() -> leds.setUserSignal(false)).ignoringDisable(true));
        driverRTButton.whileTrue(new AlgaeCommand(algae, 1));
        // driverLBButton.onTrue(new AlignToBargeCommand(driveBase)
        //     .alongWith(superstructure.getSetpointCommand(RobotConstants.BARGE_STATE))
        //     .andThen(new AlgaeCommand(algae, 1)).withTimeout(0.5)
        //     .andThen(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE))); //DON'T UNCOMMENT THIS UNTIL AFTER YOUVE TESTED LINE 246

        driverLeftPaddle.whileTrue(new AlignToReefCommand(driveBase, true));
        driverRightPaddle.whileTrue(new AlignToReefCommand(driveBase, false));

        // driverXButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L2_STATE));
        // driverYButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L3_STATE);
        // driverBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L4_STATE));

        // driverDpadDown.whileTrue(new SuperstructureStateCommand(superstructure, RobotConstants.L2_ALGAE_STATE));
        // driverDpadUp.whileTrue(new SuperstructureStateCommand(superstructure, RobotConstants.L3_ALGAE_STATE));

        //operator
        operatorLTButton.whileTrue(new CoralCommand(coral, () -> coralSpeed));
        operatorRTButton.whileTrue(new CoralCommand(coral, 0.5));
        operatorRBButton.onTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE));
        operatorLBButton.whileTrue(new AlgaeCommand(algae, 1));

        operatorXButton.onTrue(superstructure.getSetpointCommand(RobotConstants.L2_STATE));
        operatorYButton.onTrue(superstructure.getSetpointCommand(RobotConstants.L3_STATE));
        operatorBButton.onTrue(superstructure.getSetpointCommand(RobotConstants.L4_STATE));

        //operatorLeftPaddle.whileTrue(new AlgaeCommand(algae, -1));
        operatorRightPaddle.whileTrue(new AlgaeCommand(algae, -1));

        operatorDpadDown.onTrue(superstructure.getSetpointCommand(RobotConstants.L2_ALGAE_STATE));
        operatorDpadDown.whileTrue(new AlgaeCommand(algae, -1));
        operatorDpadUp.onTrue(superstructure.getSetpointCommand(RobotConstants.L3_ALGAE_STATE));
        operatorDpadUp.whileTrue(new AlgaeCommand(algae, -1));
        operatorDpadLeft.onTrue(superstructure.getSetpointCommand(RobotConstants.BARGE_STATE));
        operatorDpadRight.onTrue(superstructure.getSetpointCommand(RobotConstants.PROCESSOR_STATE));
    }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }

    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {
        autoChooser.addRoutine("Leave", List.of(), autoFactory::getLeaveAuto);

        autoChooser.addRoutine("Characterize", List.of(
                new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase, "Elevator", superstructure)),
                new AutoQuestion<>("Which Routine",
                        Map.of("Quasistatic Forward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD, "Dynamic Forward",
                                CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);

        LoggedNetworkString autoInput = new LoggedNetworkString("SmartDashboard/AutoPipes", "");

        autoChooser.addRoutine(
                "L4 Auto", List.of(
                        new AutoQuestion<>("Starting Postion",
                                Map.of("Left side left cage", StartingPosition.START_LL, "Left side middle cage",
                                        StartingPosition.START_LC, "Left side right cage", StartingPosition.START_LR,
                                        "Right side left age", StartingPosition.START_RL, "Right side middle cage",
                                        StartingPosition.START_RC, "Right side right cage", StartingPosition.START_RR)),
                        new AutoQuestion<>("Coral Station",
                                Map.of("Left", CoralStation.LEFT, "Right", CoralStation.RIGHT))),
                autoFactory.getChosenAuto(autoInput::get));

        autoChooser.addRoutine("Simple timed 1 piece", List.of(), autoFactory::getSimpleTimedAuto);

        autoChooser.addRoutine("hard-coded 2 piece", List.of(), autoFactory::getTwoPieceHardCodedAuto);

        autoChooser.addRoutine("zero", List.of(), superstructure::getZeroCommand);
    }

    public void displaySimField() {
        if (Robot.isReal()) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    }

    public void setIdleMode(boolean isBrakeMode) {
        driveBase.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        superstructure.setIdleMode(isBrakeMode);
        coral.setIdleMode(isBrakeMode);
        algae.setIdleMode(isBrakeMode);
    }

    public void periodic() {
        LEDs.getInstance().setReadyForIntake(superstructure.getState() == RobotConstants.INTAKE_STATE
                && superstructure.atSetpoint() && coral.getVelocity() > LEDConstants.INTAKE_VELOCITY_THRESHOLD);

        var mirroredRobotPose = AlliancePoseMirror.mirrorPose2d(driveBase.getPose());

        boolean aligned = false;
        for (Pose2d reefPose : Poses.REEF_POSES) {
            Transform2d poseDelta = reefPose.minus(mirroredRobotPose);
            double translationDistance = poseDelta.getTranslation().getNorm();
            double angleDifference = Math.abs(poseDelta.getRotation().getDegrees());
            if (translationDistance < LEDConstants.ALIGNED_DISTANCE && angleDifference < LEDConstants.ALIGNED_ANGLE) {
                aligned = true;
                break;
            }
        }

        LEDs.getInstance().setAligned(aligned);
        if (DriverStation.isEnabled()) driverJoystick.setRumble(RumbleType.kBothRumble, aligned ? 0.5 : 0);

        coralSpeed = superstructure.getPivotRotation().getRadians() < 1.5 ? -0.4 : -0.75;
    }
}
