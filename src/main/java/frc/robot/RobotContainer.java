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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.AutoFactory.CoralStation;
import frc.robot.AutoFactory.StartingPosition;
import frc.robot.Constants.CoralEndEffectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants.DriverIOConstants;
import frc.robot.Constants.IOConstants.OperatorIOConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.driveCommands.SwerveDriveCommand;
import frc.robot.commands.superstructureCommands.PivotPositionCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.endEffector.coral.CoralEndEffector;
import frc.robot.subsystems.endEffector.coral.CoralEndEffectorIOSparkMax;
import frc.robot.subsystems.superstructure.ElevatorIOSim;
import frc.robot.subsystems.superstructure.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.PivotIOSim;
import frc.robot.subsystems.superstructure.PivotIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.util.auto.AutonSelector;
import frc.robot.util.auto.AutonSelector.AutoQuestion;

public class RobotContainer {
    private final DriveBase driveBase;

    //sticks
    private final Joystick driverJoystick = new Joystick(DriverIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(OperatorIOConstants.OPERATOR_CONTROLLER_PORT);

    //Coral Scor
    private final JoystickButton scoreButton = new JoystickButton(driverJoystick, DriverIOConstants.SCORE_BUTTON);
    private final JoystickButton adjustButton = new JoystickButton(operatorJoystick, OperatorIOConstants.SCORE_BUTTON);
    private final JoystickButton intakeButton = new JoystickButton(operatorJoystick, OperatorIOConstants.SPIN_INTAKE_BUTTON_ID);
    //private final JoystickButton stowButton = new JoystickButton(operatorJoystick, OperatorIOConstants.STOW_BUTTON_ID);
    private final JoystickButton l1Button = new JoystickButton(operatorJoystick, OperatorIOConstants.L1_BUTTON);
    private final JoystickButton l2Button = new JoystickButton(operatorJoystick, OperatorIOConstants.L2_BUTTON);

    private final Trigger manualArm = new Trigger(
            () -> operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS) > 0.1);

    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;

    private final Superstructure superstructure;

    private final CoralEndEffector coral;

    private SwerveDriveSimulation driveSimulation = null;

    private int scoreLevel = 1;

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

            List<Camera> cameras = new ArrayList<>();
            cameras.add(new Camera(new CameraIOPhoton(VisionConstants.FRONT_CAMERA_NAME)));
            // cameras.add(new Camera(new CameraIOPhoton(VisionConstants.REAR_CAMERA_NAME)));
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

            List<Camera> cameras = new ArrayList<>();
            if (SimConstants.VISION_SIM) {
                cameras.add(new Camera(new CameraIOSim(VisionConstants.FRONT_CAMERA_NAME)));
                // cameras.add(new Camera(new CameraIOSim(VisionConstants.REAR_CAMERA_NAME)));
            }
            driveBase = new DriveBase(new GyroIOSim(driveSimulation.getGyroSimulation()) {}, cameras, frontLeft,
                    frontRight, backLeft, backRight, false);

            superstructure = new Superstructure(new ElevatorIOSim(), new PivotIOSim());
        }

        coral = new CoralEndEffector(new CoralEndEffectorIOSparkMax(CoralEndEffectorConstants.LEFT_ID));

        this.autoFactory = new AutoFactory(driveBase, coral, superstructure, autoChooser::getResponses);

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();
        
    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(
                new SwerveDriveCommand(driveBase, () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS),
                        () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                        () -> -driverJoystick.getRawAxis(DriverIOConstants.ROTATION_AXIS),
                        () -> DriveConstants.FIELD_CENTRIC, DriverIOConstants.SQUARE_INPUTS));
        // superstructure.setDefaultCommand(new PivotCommand(superstructure, () -> driverJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS)));
        superstructure.setDefaultCommand(new PivotPositionCommand(superstructure, () -> Rotation2d.fromRotations(superstructure.getPivotRotation().getRotations() + PivotConstants.JOYSTICK_SCALING * MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorIOConstants.MANUAL_ARM_AXIS), 0.1))));
        //  coral.setDefaultCommand(coral.spinCommand(CoralEndEffectorConstants.MOTOR_SPEED)
                //  .until(() -> coral.getCurrent() > CoralEndEffectorConstants.CURRENT_THRESHOLD)
                //  .andThen(new RunCommand(() -> {
                //  })));
        coral.setDefaultCommand(coral.stopCommand());
            
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
        // scoreButton.onTrue(new SelectCommand<Integer>(
        //         Map.ofEntries(Map.entry(1, new SuperstructureStateCommand(superstructure, RobotConstants.L1_STATE)),
        //                 Map.entry(2, new SuperstructureStateCommand(superstructure, RobotConstants.L2_STATE))),
        //         () -> scoreLevel)
        //                 .andThen(new CoralCommand(coral, -CoralEndEffectorConstants.MOTOR_SPEED).withTimeout(1)));
        /*
        scoreButton
                .whileTrue(new SelectCommand<Integer>(
                        Map.ofEntries(Map.entry(1, superstructure.setStateCommand(RobotConstants.L1_STATE)),
                                Map.entry(2, superstructure.setStateCommand(RobotConstants.L2_STATE))),
                        () -> scoreLevel)
                                .andThen(new CoralCommand(coral, -CoralEndEffectorConstants.MOTOR_SPEED).withTimeout(1)));
        l1Button.onTrue(new StartEndCommand(() -> scoreLevel = 1, () -> {
        }));
        l2Button.onTrue(new StartEndCommand(() -> scoreLevel = z2, () -> {
        }));
        */

        scoreButton.whileTrue(coral.spinCommand(1));
        adjustButton.whileTrue(coral.spinCommand(1));
        intakeButton.whileTrue(coral.spinCommand(-1));
        // // stowButton.whileTrue(new PivotPositionCommand(superstructure, PivotConstants.INTAKE_SETPOINT_ANGLE));
        l1Button.whileTrue(new PivotPositionCommand(superstructure, PivotConstants.INTAKE_SETPOINT_ANGLE));
        l2Button.whileTrue(new PivotPositionCommand(superstructure, PivotConstants.L2_ANGLE));
    }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }

    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {
        autoChooser.addRoutine("Characterize",
                List.of(new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase)),
                        new AutoQuestion<>("Which Routine",
                                Map.of("Quasistatic Foward", CharacterizationRoutine.QUASISTATIC_FORWARD,
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
