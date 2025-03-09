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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.Constants.IOConstants.ControllerIOConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.algaeEndEffector.AlgaeCommand;
import frc.robot.commands.algaeEndEffector.StopAlgaeCommand;
import frc.robot.commands.coralEndEffectorCommands.CoralCommand;
import frc.robot.commands.coralEndEffectorCommands.StopCoralCommand;
import frc.robot.commands.drivebase.SwerveDriveCommand;
import frc.robot.commands.superstructure.ElevatorPositionCommand;
import frc.robot.commands.superstructure.PivotPositionCommand;
import frc.robot.commands.superstructure.SuperstructureStateCommand;
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

public class RobotContainer {
    private final DriveBase driveBase;
    private final Superstructure superstructure;
    private final CoralEndEffector coral;
    private final AlgaeEndEffector algae;

    //sticks
    private final Joystick driverJoystick = new Joystick(ControllerIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(ControllerIOConstants.OPERATOR_CONTROLLER_PORT);

    //Driver
    private final Trigger driverLTButton = new Trigger(() -> driverJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5);
    private final Trigger driverRTButton = new Trigger(() -> driverJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5);

    private final JoystickButton driverLBButton = new JoystickButton(driverJoystick, ControllerIOConstants.LB_BUTTON);
    private final JoystickButton driverRBButton = new JoystickButton(driverJoystick, ControllerIOConstants.RB_BUTTON);
    
    private final JoystickButton driverLeftPaddle = new JoystickButton(driverJoystick, ControllerIOConstants.LEFT_PADDLE);
    private final JoystickButton driverRightPaddle = new JoystickButton(driverJoystick, ControllerIOConstants.RIGHT_PADDLE);

    //Operator
    private final Trigger operatorLTButton = new Trigger(() -> operatorJoystick.getRawAxis(ControllerIOConstants.LT_BUTTON) > 0.5);
    private final Trigger operatorRTButton = new Trigger(() -> operatorJoystick.getRawAxis(ControllerIOConstants.RT_BUTTON) > 0.5);

    private final JoystickButton operatorLBButton = new JoystickButton(operatorJoystick, ControllerIOConstants.LB_BUTTON);
    private final JoystickButton operatorRBButton = new JoystickButton(operatorJoystick, ControllerIOConstants.RB_BUTTON);

    private final JoystickButton operatorXButton = new JoystickButton(operatorJoystick, ControllerIOConstants.X_BUTTON);
    private final JoystickButton operatorYButton = new JoystickButton(operatorJoystick, ControllerIOConstants.Y_BUTTON);
    private final JoystickButton operatorBButton = new JoystickButton(operatorJoystick, ControllerIOConstants.B_BUTTON);

    private final POVButton operatorDpadUp = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_UP);
    private final POVButton opeartorDpadDown = new POVButton(operatorJoystick, ControllerIOConstants.D_PAD_DOWN);

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

        coral = new CoralEndEffector(
                new CoralEndEffectorIOSparkMax(CoralEndEffectorConstants.LEFT_ID));
        algae = new AlgaeEndEffector(new AlgaeEndEffectorIOSparkMax(AlgaeEndEffectorConstants.MOTOR_ID));

        this.autoFactory = new AutoFactory(driveBase, coral, superstructure, autoChooser::getResponses);

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();

    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(
                new SwerveDriveCommand(driveBase, () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL),
                        () -> -driverJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_HORIZONTAL),
                        () -> -driverJoystick.getRawAxis(ControllerIOConstants.RIGHT_STICK_HORIZONTAL),
                        () -> DriveConstants.FIELD_CENTRIC, ControllerIOConstants.SQUARE_INPUTS));
        // superstructure.setDefaultCommand(new PivotCommand(superstructure, () -> driverJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS)));
        // superstructure.setDefaultCommand(new PivotPositionCommand(superstructure, () -> Rotation2d.fromRotations(
        //         superstructure.getPivotRotation().getRotations() + PivotConstants.JOYSTICK_SCALING * MathUtil
        //                 .applyDeadband(-operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS), 0.1))));
        // superstructure.setDefaultCommand(new SuperstructureStateCommand(superstructure, () -> Rotation2d.fromRotations(superstructure.getPivotRotation().getRotations() + PivotConstants.JOYSTICK_SCALING * MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS), 0.1))));
        // superstructure.setDefaultCommand(superstructure.run(() -> {
        //     superstructure.setRotation(superstructure.getPivotRotation().plus(Rotation2d.fromRotations(PivotConstants.JOYSTICK_SCALING * MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS), 0.1))));
        //     superstructure.setExtension(superstructure.getExtension() + MathUtil.applyDeadband(operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ELEVATOR_AXIS), 0.1), 0);
        // }));
        // superstructure.setDefaultCommand(Commands.run(() -> superstructure.setState(superstructure.getState()), superstructure));
        //  coral.setDefaultCommand(coral.spinCommand(CoralEndEffectorConstants.MOTOR_SPEED)
        //  .until(() -> coral.getCurrent() > CoralEndEffectorConstants.CURRENT_THRESHOLD)
        //  .andThen(new RunCommand(() -> {
        //  })));
        SmartDashboard.putData("thing", superstructure);
        coral.setDefaultCommand(new StopCoralCommand(coral));
        algae.setDefaultCommand(new StopAlgaeCommand(algae));
        superstructure.setDefaultCommand(Commands.run(() -> superstructure.setState(
            new SuperstructureState(
            superstructure.getGoal().pivotRotation.plus(Rotation2d.fromRadians(0.1 * operatorJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL))),
            superstructure.getGoal().elevatorHeight + operatorJoystick.getRawAxis(ControllerIOConstants.RIGHT_STICK_VERTICAL) * -2, 0, 0
            )), superstructure));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SwerveDriveCommand(driveBase, -1, 0, 0, true, false).withTimeout(2);
        // return autoChooser.getCommand();
    }

    public void configureButtonBindings() {

        //driver
        driverLTButton.whileTrue(new CoralCommand(coral, -0.75));
        driverRTButton.onTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE).andThen(new CoralCommand(coral, -0.5)));
        driverLBButton.whileTrue(new AlgaeCommand(algae, 1));
        driverRBButton.whileTrue(new CoralCommand(coral, 0.5));
        //driverRBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE)); //NOTE: Fancy collision avoidance that is untested and doesn't necessarily work and is messing things up
        
        // driverXButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L2_STATE));
        // driverYButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L3_STATE);
        // driverBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L4_STATE));
        
        // driverDpadDown.whileTrue(new SuperstructureStateCommand(superstructure, RobotConstants.L2_ALGAE_STATE));
        // driverDpadUp.whileTrue(new SuperstructureStateCommand(superstructure, RobotConstants.L3_ALGAE_STATE));

        // driverLeftPaddle.onTrue(new AlignToReefCommand(driveBase, false));
        // driverRightPaddle.onTrue(new AlignToReefCommand(driveBase, true));

        //operator
        operatorLTButton.whileTrue(new CoralCommand(coral, -0.5));
        operatorRTButton.whileTrue(new CoralCommand(coral, 0.5));
        operatorLBButton.whileTrue(new AlgaeCommand(algae, -0.75))
        ;
        operatorRBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.INTAKE_STATE));
        
        operatorXButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L2_STATE));
        operatorYButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L3_STATE));
        operatorBButton.whileTrue(superstructure.getSetpointCommand(RobotConstants.L4_STATE));
        
        // TODO: uncoment after the algae states are actually inputed properly
        // opeartorDpadDown.whileTrue(new SuperstructureStateCommand(supers                  tructure, RobotConstants.L2_ALGAE_STATE));
        // operatorDpadUp.whileTrue(new SuperstructureStateCommand(superstructure, RobotConstants.L3_ALGAE_STATE));

        // //TODO: Fix manual mode
        // manualArm.whileTrue(new PivotPositionCommand(superstructure, () -> Rotation2d.fromRotations(0.5 * operatorJoystick.getRawAxis(ControllerIOConstants.LEFT_STICK_VERTICAL))));
        // manualArm.whileTrue(new ElevatorPositionCommand(superstructure, () -> (superstructure.getState().elevatorHeight + 3 * operatorJoystick.getRawAxis(ControllerIOConstants.RIGHT_STICK_VERTICAL))));
    }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }

    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {
        autoChooser.addRoutine("Characterize",
                List.of(new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase, "Elevator", superstructure)),
                        new AutoQuestion<>("Which Routine",
                                Map.of("Quasistatic Forward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                        "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD,
                                        "Dynamic Forward", CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                        CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);

        LoggedNetworkString autoInput = new LoggedNetworkString("SmartDashboard/AutoPipes", "");

        autoChooser.addRoutine(
                "L2 Auto", List.of(
                        new AutoQuestion<>("Starting Postion",
                                Map.of("Left side left cage", StartingPosition.START_LL, "Left side middle cage",
                                        StartingPosition.START_LC, "Left side right cage", StartingPosition.START_LR,
                                        "Right side left age", StartingPosition.START_RL, "Right side middle cage",
                                        StartingPosition.START_RC, "Right side right cage", StartingPosition.START_RR)),
                        new AutoQuestion<>("Coral Station",
                                Map.of("Left", CoralStation.LEFT, "Right", CoralStation.RIGHT))),
                autoFactory.getChosenAuto(autoInput::get));
    }

    public void displaySimField() {
        if (Robot.isReal()) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    }
}
