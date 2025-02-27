package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralEndEffectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.coralEndEffectorCommands.CoralCommand;
import frc.robot.commands.driveCommands.SwerveDriveStopCommand;
import frc.robot.commands.superstructureCommands.SuperstructureStateCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.endEffector.coral.CoralEndEffector;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.choreo.ChoreoVariables;
import frc.robot.util.sysId.CharacterizableSubsystem;

public class AutoFactory {
    private final Supplier<List<Object>> responses;
    private final DriveBase driveBase;
    private final CoralEndEffector coral;
    private final Superstructure superstructure;

    /**
     * Create a new auto factory.
     * 
     * @param driveBase         {@link DriveBase} to drive.
     * @param responsesSupplier Responses to auto chooser questions.
     * @see frc.robot.util.auto.AutonSelector
     */
    public AutoFactory(DriveBase driveBase, CoralEndEffector coral, Superstructure superstructure,
            Supplier<List<Object>> responsesSupplier) {
        this.responses = responsesSupplier;
        this.driveBase = driveBase;
        this.coral = coral;
        this.superstructure = superstructure;

        AutoBuilder.configure(driveBase::getPose, driveBase::resetPose, driveBase::getRobotRelativeSpeeds,
                (chassisSpeeds, driveFeedforwards) -> driveBase.driveRobotRelative(chassisSpeeds),
                new PPHolonomicDriveController(DriveConstants.TRANSLATION_PID_CONSTANTS,
                        DriveConstants.ROTATION_PID_CONSTANTS),
                DriveConstants.ROBOT_CONFIG, () -> {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }, driveBase);
    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO, PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, PathConstants.CONSTRAINTS, 0.0 // Goal end velocity in meters/sec
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose Supplier for the desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Supplier<Pose2d> targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose.get(), PathConstants.CONSTRAINTS, 0.0 // Goal end velocity in meters/sec
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @param segment  The segment of the Choreo path to choose, zero-indexed
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType, int segment) {
        PathPlannerPath path;
        try {
            switch (pathType) {
                case CHOREO:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname, segment);
                    break;
                case PATHPLANNER:
                    path = PathPlannerPath.fromPathFile(pathname);
                    break;
                default:
                    path = PathPlannerPath.fromChoreoTrajectory(pathname, segment);
            }

            return AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS);
        } catch (Exception exception) {
            DriverStation.reportError("Could not load path " + pathname + ". Error: " + exception.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed. If it is a Choreo
     * path, get the first split segment.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType) {
        return getPathFindToPathCommand(pathname, pathType, 0);
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends
     * with desired holonomic rotation.
     * 
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses                    List of bezier poses. Each {@link Pose2d}
     *                                 represents one waypoint. The rotation
     *                                 component of the pose should be the direction
     *                                 of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with
     *         set end rotation.
     */
    public Supplier<Command> getPathFromWaypoints(Rotation2d goalEndRotationHolonomic, Pose2d... poses) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(waypoints, PathConstants.CONSTRAINTS, null,
                new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        Supplier<Command> pathCommand = () -> AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS);
        return pathCommand;
    }

    /**
     * Which coral station we are using
     */
    public static enum CoralStation {
        /**
         * The left-hand coral station
         */
        LEFT,
        /**
         * The right-hand-coral station
         */
        RIGHT
    }

    /**
     * Represents the starting position of a robot.
     */
    public static enum StartingPosition {
        /*
         * Pose on the left-most cage, with bumpers but not robot perimeter intersecting
         * the auto start line, facing the driver station wall.
         */
        START_LL(ChoreoVariables.getPose("START_LL")),
        /**
         * Pose on the center cage on the left, with bumpers but not robot perimeter
         * intersecting the auto start line, facing the driver station wall.
         */
        START_LC(ChoreoVariables.getPose("START_LC")),
        /**
         * Pose on the left cage closest to the center, with bumpers but not robot
         * perimeter intersecting the auto start line, facing the driver station wall.
         */
        START_LR(ChoreoVariables.getPose("START_LR")),
        /**
         * Pose on the right cage closest to the center, with bumpers but not robot
         * perimeter intersecting the auto start line, facing the driver station wall.
         */
        START_RL(ChoreoVariables.getPose("START_RL")),
        /**
         * Pose on the center cage on the right, with bumpers but not robot perimeter
         * intersecting the auto start line, facing the driver station wall.
         */
        START_RC(ChoreoVariables.getPose("START_RC")),
        /**
         * Pose on the right-most cage, with bumpers but not robot perimeter
         * intersecting the auto start line, facing the driver station wall.
         */
        START_RR(ChoreoVariables.getPose("START_RR"));

        public final Pose2d pose;

        private StartingPosition(Pose2d pose) {
            this.pose = pose;
        }
    }

    /**
     * Get the command to go and score the first coral in auto.
     * 
     * @param startingPosition the starting position the robot is in; odometry pose
     *                         will be automatically reset to this at the beginning
     *                         of auto
     * @param pipe             the pipe to score on, as a character ('A', 'C', 'G',
     *                         et cetera)
     * @return The constructed command
     */
    public Command getStartCommand(StartingPosition startingPosition, char pipe) {
        return getPathFindToPathCommand(startingPosition.name() + "_" + pipe, PathType.CHOREO)
                .alongWith(new SuperstructureStateCommand(superstructure, RobotConstants.L2_STATE))
                .andThen(new CoralCommand(coral, CoralEndEffectorConstants.MOTOR_SPEED).withTimeout(1));
    }

    /**
     * Get the command to go back and intake a coral
     * 
     * @param coralStation coral station to go to
     * @param pipe         pipe to go from
     * @return The constructed command
     */
    public Command getCoralStationCommand(CoralStation coralStation, char pipe) {
        return getPathFindToPathCommand(coralStation.name() + "_" + pipe, PathType.CHOREO, 1)
                .alongWith(Commands.waitSeconds(0.5)
                        .andThen(new SuperstructureStateCommand(superstructure, RobotConstants.INTAKE_STATE)))
                .andThen(new CoralCommand(coral, -CoralEndEffectorConstants.MOTOR_SPEED).withTimeout(1));
    }

    /**
     * Get the command to score a coral from a coral station
     * 
     * @param coralStation the coral station to start at
     * @param pipe         the pipe to end at
     * @return the constructed command
     */
    public Command getScoreCommand(CoralStation coralStation, char pipe) {
        return getPathFindToPathCommand(coralStation.name() + "_" + pipe, PathType.CHOREO, 0)
                .alongWith(new SuperstructureStateCommand(superstructure, RobotConstants.L2_STATE))
                .andThen(new CoralCommand(coral, CoralEndEffectorConstants.MOTOR_SPEED).withTimeout(1));
    }

    /**
     * Construct a command to do an auto routine
     * 
     * @param startingPosition the position to start at
     * @param coralStation     the coral station to use
     * @param pipes            pipes, as a string, e.g. "ALG" or "FBC"
     * @return the constructed command
     */
    public Command getAuto(StartingPosition startingPosition, CoralStation coralStation, String pipes) {
        if (pipes.length() == 0) return Commands.none();
        Command result = getStartCommand(startingPosition, pipes.charAt(0));
        for (int i = 1; i < pipes.length(); i++) {
            result = result.andThen(getCoralStationCommand(coralStation, pipes.charAt(i - 1)))
                    .andThen(getScoreCommand(coralStation, pipes.charAt(i)));
        }
        return result;
    }

    /**
     * For use in auto chooser
     * 
     * @param pipes supplier for pipes
     * @return supplier to construct command
     */
    public Supplier<Command> getChosenAuto(Supplier<String> pipes) {
        return () -> getAuto((StartingPosition) responses.get().get(0), (CoralStation) responses.get().get(1),
                pipes.get());
    }

    public static enum CharacterizationRoutine {
        QUASISTATIC_FORWARD, QUASISTATIC_BACKWARD, DYNAMIC_FORWARD, DYNAMIC_BACKWARD,
    }

    public Command getCharacterizationRoutine() {
        CharacterizableSubsystem subsystem = (CharacterizableSubsystem) responses.get().get(0);
        CharacterizationRoutine routine = (CharacterizationRoutine) responses.get().get(1);

        var sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((Voltage voltage) -> subsystem.runVolts(voltage.in(Volts)), null, // No log consumer, since data is recorded by AdvantageKit
                        subsystem));
        switch (routine) {
            case QUASISTATIC_FORWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
            case QUASISTATIC_BACKWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
            case DYNAMIC_FORWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
            case DYNAMIC_BACKWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
            default:
                return new WaitCommand(1);
        }
    }
}
