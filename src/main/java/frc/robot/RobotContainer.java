// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.IOConstants.DriverIOConstants;
import frc.robot.Constants.IOConstants.OperatorIOConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import stl.auto.AutonSelector;
import stl.auto.AutonSelector.AutoQuestion;

import java.util.List;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final DriveBase driveBase;
    private final Joystick driverJoystick = new Joystick(DriverIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(OperatorIOConstants.OPERATOR_CONTROLLER_PORT);
    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;

    private SwerveDriveSimulation driveSimulation = null;

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

            driveBase = new DriveBase(new GyroIONavX(), new Vision(new VisionIOPhoton()), frontLeft, frontRight,
                    backLeft, backRight, false);
        } else {
            driveSimulation = new SwerveDriveSimulation(DriveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

            var modules = driveSimulation.getModules();
            SwerveModuleIOSim frontLeft = new SwerveModuleIOSim(FrontLeftModuleConstants.angleOffset, modules[0]);
            SwerveModuleIOSim frontRight = new SwerveModuleIOSim(FrontRightModuleConstants.angleOffset, modules[1]);
            SwerveModuleIOSim backLeft = new SwerveModuleIOSim(BackLeftModuleConstants.angleOffset, modules[2]);
            SwerveModuleIOSim backRight = new SwerveModuleIOSim(BackRightModuleConstants.angleOffset, modules[3]);

            driveBase = new DriveBase(new GyroIOSim(driveSimulation.getGyroSimulation()) {}, new Vision(new VisionIOSim()), frontLeft, frontRight, backLeft,
                    backRight, false);
        }

        this.autoFactory = new AutoFactory(driveBase, autoChooser::getResponses);

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();
    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(new SwerveDriveCommand(driveBase,
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS),
                () -> driverJoystick.getRawAxis(DriverIOConstants.ROTATION_AXIS), () -> DriveConstants.FIELD_CENTRIC, DriverIOConstants.SQUARE_INPUTS));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile("New New Path"));
        } catch (Exception exception) {
            return new RunCommand(() -> {});
        }
    }

    public void configureButtonBindings() {
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
    }

    public void displaySimField() {
        if (Robot.isReal()) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    }
}
