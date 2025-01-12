// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.drive.SwerveKinematicLimits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PathConstants {
    public static final Pose2d TARGET_POSE = new Pose2d(16, 7, Rotation2d.fromDegrees(180));
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final PathConstraints CONSTRAINTS = new PathConstraints(1, 1, Units.degreesToRadians(540),
        Units.degreesToRadians(720));

    public static final Pose2d STATION_1 = new Pose2d(0.4119143784046173, 7.161474227905273,
        Rotation2d.fromRotations(0));
    public static final Pose2d STATION_2 = new Pose2d(0.5068893432617188, 3.710716009140014,
        Rotation2d.fromRotations(0));
    public static final Pose2d STATION_3 = new Pose2d(0.44357267022132874, 2.3525, Rotation2d.fromRotations(0));
  }

  public static class IOConstants {
    public static final double JOYSTICK_DEADBAND = 0.1;

    public static class DriverIOConstants {
      public static final int DRIVER_CONTROLLER_PORT = 0;
      public static final int STRAFE_X_AXIS = 0;
      public static final int STRAFE_Y_AXIS = 1;
      public static final int ROTATION_AXIS = 2;
      public static final boolean SQUARE_INPUTS = false;
    }

    public static class OperatorIOConstants {
      public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
  }

  public static class RobotConstants {
    public static final double WHEELBASE = Units.inchesToMeters(20);
    public static final double TRACK_WIDTH = Units.inchesToMeters(20);
    public static final double EDGE_TO_MODULE_CENTER = Units.inchesToMeters(1.75);
    // Distance from robot center to module center
    public static final double RADIUS = Math.sqrt(2 * Math.pow(WHEELBASE / 2 - EDGE_TO_MODULE_CENTER, 2));
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double DRIVE_GEAR_RATIO = 4.71;
    public static final double ANGLE_GEAR_RATIO = 9424 / 203;
    public static final double MAX_DRIVE_SPEED = 5.23; // from https://www.reca.lc/drive
    public static final Mass WEIGHT = Pounds.of(40);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.4988172);
  }

  public static class DriveConstants {
    public static final double MAX_ACCELERATION = 30;
    public static final double MAX_DRIVE_SPEED = 100;
    public static final double MAX_ANGULAR_SPEED = 100;
    public static final double SLOWDOWN_PERCENT = 0.5;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int ANGLE_MOTOR_CURRENT_LIMIT = 40;
    public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
        new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
        new Translation2d(RobotConstants.WHEELBASE / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER,
            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER),
        new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
            RobotConstants.TRACK_WIDTH / 2.0 - RobotConstants.EDGE_TO_MODULE_CENTER),
        new Translation2d(-RobotConstants.WHEELBASE / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER,
            -RobotConstants.TRACK_WIDTH / 2.0 + RobotConstants.EDGE_TO_MODULE_CENTER), };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);
    public static final SwerveKinematicLimits MODULE_LIMITS = new SwerveKinematicLimits(MAX_DRIVE_SPEED,
        MAX_ACCELERATION, MAX_ANGULAR_SPEED);

    public static boolean FIELD_CENTRIC = true;
    public static final boolean IS_OPEN_LOOP = false;

    public static final double PATH_MAX_ACCEL = 3;
    public static final double PATH_MAX_VELOCITY = 3;

    public static final double TURN_KP = 2;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;

    public static final double TURN_DEADBAND = Units.degreesToRadians(5);

    public static final double WHEEL_COF = 1;

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(RobotConstants.WEIGHT, // Robot mass
        RobotConstants.MOI, // Robot moment of inertia
        new ModuleConfig(RobotConstants.WHEEL_DIAMETER / 2, // wheel diameter
            RobotConstants.MAX_DRIVE_SPEED, // max drive velocity (m/s)
            WHEEL_COF, // cof between wheels and ground
            DCMotor.getNEO(1).withReduction(RobotConstants.DRIVE_GEAR_RATIO), // DCMotor representing motor, including reduction
            DRIVE_MOTOR_CURRENT_LIMIT, // current limit for drive motors
            1 // number of drive motors per module
        ), MODULE_LOCATIONS);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(1, 0.0, 0);
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(2, 0.0, 0);

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(MODULE_LOCATIONS).withGyro(COTS.ofNav2X()).withRobotMass(RobotConstants.WEIGHT)
        .withSwerveModule(COTS.ofMAXSwerve(DCMotor.getNEO(1), DCMotor.getNeo550(1), WHEEL_COF, 3));

    public static class FrontLeftModuleConstants {
      public static final int moduleID = 0;
      public static final int driveID = 14;
      public static final int angleID = 15;
      public static final double angleOffset = -90;
      public static final boolean inverted = false;
    }

    public static class BackRightModuleConstants {
      public static final int moduleID = 3;
      public static final int driveID = 11;
      public static final int angleID = 10;
      public static final double angleOffset = 90;
      public static final boolean inverted = false;
    }

    public static class FrontRightModuleConstants {
      public static final int moduleID = 1;
      public static final int driveID = 16;
      public static final int angleID = 17;
      public static final double angleOffset = 0;
      public static final boolean inverted = false;
    }

    public static class BackLeftModuleConstants {
      public static final int moduleID = 2;
      public static final int driveID = 13;
      public static final int angleID = 12;
      public static final double angleOffset = 180;
      public static final boolean inverted = false;
    }
  }

  public static class SwerveConstants {
    public static final boolean invertGyro = true;

    public static final double KS = 0.1;
    public static final double KA = 0.1;
    public static final double KV = 0.1;

    public static final double ANGLE_KS = 0.1;
    public static final double ANGLE_KA = 0.1;
    public static final double ANGLE_KV = 0.1;

    public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 1 / RobotConstants.DRIVE_GEAR_RATIO;
    public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = DRIVING_ENCODER_POSITION_CONVERSION_FACTOR
        / 60.0;
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI);
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR
        / 60.0;

    public static final double TURN_PID_MIN_INPUT = -Math.PI;
    public static final double TURN_PID_MAX_INPUT = Math.PI;

    public static final double DRIVE_PID_MIN_OUTPUT = -1;
    public static final double DRIVE_PID_MAX_OUTPUT = 1;
    public static final double DRIVE_PID_P = 0.05;
    public static final double DRIVE_PID_I = 0.02;
    public static final double DRIVE_PID_D = 0;
    public static final double DRIVE_PID_FF = 0;

    public static final double TURN_PID_MIN_OUTPUT = -2 * Math.PI;
    public static final double TURN_PID_MAX_OUTPUT = 2 * Math.PI;
    public static final double TURN_PID_P = 2;
    public static final double TURN_PID_I = 0;
    public static final double TURN_PID_D = 0;
    public static final double TURN_PID_FF = 0;

    public static final double AIM_VELOCITY_COMPENSATION_DEADBAND = 0.3;
  }

  public static class SimConstants {
    public static final double LOOP_TIME = 0.02;
    public static final boolean REPLAY = false;
    public static final String REPLAY_LOG_PATH = "Log_24-03-10_09-23-09_q61.wpilog";
  }

  public static class VisionConstants {
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(Units.inchesToMeters(13.916),
        Units.inchesToMeters(3.102475), Units.inchesToMeters(7.820), new Rotation3d(0, Units.degreesToRadians(-35), 0));
    public static final Transform3d ROBOT_TO_REAR_CAMERA = new Transform3d(Units.inchesToMeters(-13.193037),
        Units.inchesToMeters(-9.543), Units.inchesToMeters(7.820),
        new Rotation3d(0, Units.degreesToRadians(-35), Units.degreesToRadians(180)));
    public static final double VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD = 5;
    public static final int CAMERA_RES_WIDTH = 1280;
    public static final int CAMERA_RES_HEIGHT = 960;
    public static final int CAMERA_FOV_DEG = 70;
    public static final double CAMERA_AVG_LATENCY_MS = 35;
    public static final double AVG_ERROR_PX = 0.25;
    public static final double ERROR_STDEV_PX = 0.08;
    public static final double FPS = 20;
    public static final double CAMERA_LATENCY_STDEV_MS = 5;

    public static final double APRIL_TAG_NUMBER_CONFIDENCE_SCALE = 3; // Higher makes confidence lower at each number of
                                                                      // AprilTags
    public static final double APRIL_TAG_NUMBER_EXPONENT = -1
        / (APRIL_TAG_NUMBER_CONFIDENCE_SCALE * Math.log(APRIL_TAG_NUMBER_CONFIDENCE_SCALE));
    public static final double APRIL_TAG_AREA_CONFIDENCE_SCALE = 1.7; // Higher makes confidence lower at each area of
                                                                      // AprilTags
                                                                      // See https://www.desmos.com/calculator/i5z7ddbjy4

    public static final double AMBIGUITY_TO_STDEV_EXP = 1;
    public static final Vector<N3> BASE_STDEV = VecBuilder.fill(0.1, 0.1, 1000.0); // x, y, angle
    public static final double AMBIGUITY_ACCEPTANCE_THRESHOLD = 0.2;
    public static final double REPROJECTION_ERROR_REJECTION_THRESHOLD = 0.4;
  }

  public static class TempConstants {
    public static final int OVERHEAT_TEMP = 80;
    public static final int SAFE_TEMP = 80;
  }

  public static class FieldConstants {
    public static final double FIELD_LENGTH = 16.54;
  }

  public static class AlertConstants {
    public static final double LOW_BATTERY_VOLTAGE = 11.5;
    public static final int ENDGAME_ALERT_1_TIME = 45;
    public static final int ENDGAME_ALERT_2_TIME = 30;
  }

  public static class LEDConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 100;
  }

  public static class LoggingConstants {
    public static final double LOG_ALERT_INTERVAL = 5; // Interval (in s) between logs of an alert if its text doesn't change
  }
}
