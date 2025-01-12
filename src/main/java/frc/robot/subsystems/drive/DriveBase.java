// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Poses;
import stl.math.LobstahMath;
import stl.sysId.CharacterizableSubsystem;

public class DriveBase extends CharacterizableSubsystem {
  /** Creates a new SwerveDriveBase. */
  private final SwerveModule[] modules;

  private SwerveDrivePoseEstimator swerveOdometry;
  private SwerveDrivePoseEstimator visionLessOdometry;
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint swerveSetpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() });
  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private boolean isOpenLoop;
  private Rotation2d simRotation = new Rotation2d();
  private final Vision photonVision;
  private boolean hasSeenTag = false;

  private Field2d field;

  /**
   * Create a new serve drivebase.
   * 
   * @param gyroIO       The {@link GyroIO}.
   * @param photonVision The {@link Vision} instance for odometry and note
   *                     detection.
   * @param frontLeft
   * @param frontRight
   * @param backLeft
   * @param backRight
   * @param isOpenLoop   If true, skip closed-loop (PID) control.
   */
  public DriveBase(GyroIO gyroIO, Vision photonVision, SwerveModuleIO frontLeft, SwerveModuleIO frontRight,
      SwerveModuleIO backLeft, SwerveModuleIO backRight, boolean isOpenLoop) {
    this.modules = new SwerveModule[] { new SwerveModule(frontLeft, FrontLeftModuleConstants.moduleID),
        new SwerveModule(frontRight, FrontRightModuleConstants.moduleID),
        new SwerveModule(backLeft, BackLeftModuleConstants.moduleID),
        new SwerveModule(backRight, BackRightModuleConstants.moduleID) };

    this.gyro = gyroIO;

    gyro.zeroGyro();
    this.photonVision = photonVision;
    swerveOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyroInputs.yawPosition, getPositions(),
        new Pose2d());
    visionLessOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyroInputs.yawPosition, getPositions(),
        new Pose2d());
    setpointGenerator = new SwerveSetpointGenerator(DriveConstants.KINEMATICS, DriveConstants.MODULE_LOCATIONS);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    this.isOpenLoop = isOpenLoop;
    this.resetPose(getPose());
  }

  /**
   * Resets pose of odometry to a given pose.
   * 
   * @param pose The desired pose to reset the odometry to.
   */
  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(gyroInputs.yawPosition, getPositions(), pose);
    visionLessOdometry.resetPosition(gyroInputs.yawPosition, getPositions(), pose);
  }

  /**
   * Gets pose from odometry.
   * 
   * @return The current estimated pose of the odometry
   */
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Gets states of the four swerve modules.
   * 
   * @return The states of the four swerve modules in a {@link SwerveModuleState}
   *         array.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : modules) {
      states[module.getModuleID()] = module.getState();
    }
    return states;
  }

  /**
   * Gets positions of the four swerve modules.
   * 
   * @return The positions of the four swerve modules in a
   *         {@link SwerveModulePosition} array.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule module : modules) {
      positions[module.getModuleID()] = module.getPosition();
    }
    return positions;
  }

  /**
   * Gets robot relative ChassisSpeeds.
   * 
   * @return The robot-relative {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.KINEMATICS.toChassisSpeeds(getStates());
  }

  /**
   * Drives the robot robot-relative according to provided {@link ChassisSpeeds}.
   * 
   * @param chassisSpeeds The desired ChassisSpeeds. Should be robot relative.
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    Logger.recordOutput("Unoptimized:", DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    swerveSetpoint = setpointGenerator.generateSetpoint(DriveConstants.MODULE_LIMITS,
        new SwerveSetpoint(getRobotRelativeSpeeds(), getStates()), ChassisSpeeds.discretize(chassisSpeeds, 0.02), SimConstants.LOOP_TIME);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveSetpoint.moduleStates, DriveConstants.MAX_DRIVE_SPEED);
    setModuleStates(swerveSetpoint.moduleStates);
  }

  /**
   * Sets desired {@link SwerveModuleState}s. Optimizes states.
   * 
   * @param desiredStates The states to set for each module.
   * @return The optimized SwerveModuleStates, now desired states.
   */
  public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    // desiredStates, DriveConstants.MAX_DRIVE_SPEED);
    for (SwerveModule module : modules) {
      optimizedStates[module.getModuleID()] = module.setDesiredState(desiredStates[module.getModuleID()], isOpenLoop);
    }
    swerveSetpoint.moduleStates = optimizedStates;
    Logger.recordOutput("SwerveStates/Desired", desiredStates);
    Logger.recordOutput("SwerveStates/Optimized", swerveSetpoint.moduleStates);
    Logger.recordOutput("SwerveStates/SetpointSpeeds", swerveSetpoint.chassisSpeeds);
    return optimizedStates;
  }

  /**
   * Converts robot relative {@link ChassisSpeeds} to field relative.
   * 
   * @param robotRelativeSpeeds The robot relative speeds to convert
   * @return The field relative ChassisSpeeds.
   */
  public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    Rotation2d angle = new Rotation2d();
    if (Robot.isSimulation()) {
      angle = simRotation;
    } else {
      angle = getPose().getRotation();
    }
    return new ChassisSpeeds(
        robotRelativeSpeeds.vxMetersPerSecond * angle.getCos() - robotRelativeSpeeds.vyMetersPerSecond * angle.getSin(),
        robotRelativeSpeeds.vyMetersPerSecond * angle.getCos() + robotRelativeSpeeds.vxMetersPerSecond * angle.getSin(),
        robotRelativeSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Sets the {@link IdleMode} of the DriveBase motors.
   * 
   * @param mode The braking mode (Coast or Brake) of the swerve module motors.
   */
  public void setIdleMode(IdleMode mode) {
    for (SwerveModule module : modules) {
      module.setIdleMode(mode);
    }
  }

  /** Stops all of the modules' motors. */
  public void stopMotors() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * @return Whether or not the controller is open loop.
   */
  public boolean isOpenLoop() {
    return isOpenLoop;
  }

  /**
   * Sets whether the controller is open loop.
   * 
   * @param newValue The new boolean to set.
   */
  public void setIsOpenLoop(boolean newValue) {
    isOpenLoop = newValue;
  }

  /**
   * Gets gyro angle
   * 
   * @return The angle of the gyro as a {@link Rotation2d}.
   */
  public Rotation2d getGyroAngle() {
    return gyroInputs.yawPosition;
  }

  @Override
  /**
   * Runs motors during characterization voltage ramp routines.
   * 
   * @param Voltage to run motors at.
   * @see CharacterizableSubsystem
   */
  public void runVolts(double volts) {
    for (SwerveModule module : modules) {
      module.runVolts(volts);
    }
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      var twist = DriveConstants.KINEMATICS.toTwist2d(getPositions());
      simRotation = gyroInputs.yawPosition.plus(Rotation2d.fromDegrees(twist.dtheta));
      SmartDashboard.putNumber("Twist Theta", twist.dtheta);
      swerveOdometry.update(simRotation, getPositions());
      visionLessOdometry.update(simRotation, getPositions());
      photonVision.update(getPose());
    } else {
      swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, getPositions());
      visionLessOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, getPositions());
    }
    SmartDashboard.putBoolean("Has seen tag", hasSeenTag);
    Poses estimatedPoses = photonVision.getEstimatedPose(getPose());
    if (estimatedPoses.frontPose().isPresent() && (hasSeenTag == false
        || LobstahMath.getDistBetweenPoses(estimatedPoses.frontPose().get().toPose2d(), getPose()) <= 1)) {
      if (hasSeenTag == false) {
        resetPose(new Pose2d(estimatedPoses.frontPose().get().getX(), estimatedPoses.frontPose().get().getY(),
            getGyroAngle()));
        hasSeenTag = true;
      }
      Logger.recordOutput("Front Pose", estimatedPoses.frontPose().get());
      Logger.recordOutput("Front Stdev", estimatedPoses.frontStdev().toString());
      swerveOdometry.addVisionMeasurement(estimatedPoses.frontPose().get().toPose2d(), Timer.getFPGATimestamp(),
          estimatedPoses.frontStdev().get());
    }
    if (estimatedPoses.rearPose().isPresent() && (hasSeenTag == false
        || LobstahMath.getDistBetweenPoses(estimatedPoses.rearPose().get().toPose2d(), getPose()) <= 1)) {
      if (hasSeenTag == false) {
        resetPose(
            new Pose2d(estimatedPoses.rearPose().get().getX(), estimatedPoses.rearPose().get().getY(), getGyroAngle()));
        hasSeenTag = true;
      }
      Logger.recordOutput("Rear Pose", estimatedPoses.rearPose().get());
      Logger.recordOutput("Rear Stdev", estimatedPoses.rearStdev().toString());
      swerveOdometry.addVisionMeasurement(estimatedPoses.rearPose().get().toPose2d(), Timer.getFPGATimestamp(),
          estimatedPoses.rearStdev().get());
    }
    resetPose(new Pose2d(
        MathUtil.clamp(getPose().getX(), RobotConstants.TRACK_WIDTH / 2, 16.5 - RobotConstants.TRACK_WIDTH / 2),
        MathUtil.clamp(getPose().getY(), RobotConstants.TRACK_WIDTH / 2, 8 - RobotConstants.TRACK_WIDTH / 2),
        getPose().getRotation()));
    field.setRobotPose(getPose());
    Logger.recordOutput("Odometry", getPose());
    Logger.recordOutput("Vision Less", visionLessOdometry.getEstimatedPosition());
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    SmartDashboard.putBoolean("Field Centric", DriveConstants.FIELD_CENTRIC);
    for (var module : modules) {
      module.periodic();
    }
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }
    }

    else {
      Logger.recordOutput("SwerveStates/Measured", getStates());
      Logger.recordOutput("Odometry/Robot", getPose());

      // Log 3D odometry pose
      Pose3d robotPose3d = new Pose3d(getPose());
      robotPose3d = robotPose3d
          .exp(new Twist3d(0.0, 0.0, Math.abs(gyroInputs.pitchPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0,
              0.0, gyroInputs.pitchPosition.getRadians(), 0.0))
          .exp(new Twist3d(0.0, 0.0, Math.abs(gyroInputs.rollPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0,
              gyroInputs.rollPosition.getRadians(), 0.0, 0.0));

      Pose3d frontLeftPose3d = new Pose3d(getPose()).plus(VisionConstants.ROBOT_TO_FRONT_CAMERA);
      Pose3d backRightPose3d = new Pose3d(getPose()).plus(VisionConstants.ROBOT_TO_REAR_CAMERA);

      Logger.recordOutput("Back Right", backRightPose3d);
      Logger.recordOutput("Front Left", frontLeftPose3d);
      Logger.recordOutput("Front Left Pose", robotPose3d.plus(VisionConstants.ROBOT_TO_FRONT_CAMERA));
      Logger.recordOutput("Back Right Pose", robotPose3d.plus(VisionConstants.ROBOT_TO_REAR_CAMERA));

      Logger.recordOutput("Odometry/Robot3d", robotPose3d);
    }
  }
}
