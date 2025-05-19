// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.FieldConstants.Poses;
import frc.robot.Constants.IOConstants.ControllerIOConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.Camera.Pose;
import frc.robot.util.led.LEDs;
import frc.robot.util.math.LobstahMath;
import frc.robot.util.sysId.CharacterizableSubsystem;
import frc.robot.util.trajectory.AlliancePoseMirror;

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
    private final List<Camera> cameras;
    private boolean hasSeenTag = false;
    private boolean needGyroReset = true;

    private final PIDController xController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
            DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);
    private final PIDController yController = new PIDController(DriveConstants.AUTO_ALIGN_TRANSLATION_kP,
            DriveConstants.AUTO_ALIGN_TRANSLATION_kI, DriveConstants.AUTO_ALIGN_TRANSLATION_kD);
    private final PIDController thetaController = new PIDController(DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI, DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private Field2d field;

    /**
     * Create a new serve drivebase.
     * 
     * @param gyroIO     The {@link GyroIO}.
     * @param cameras    a list of {@link Camera}s on the robot
     * @param frontLeft
     * @param frontRight
     * @param backLeft
     * @param backRight
     * @param isOpenLoop If true, skip closed-loop (PID) control.
     */
    public DriveBase(GyroIO gyroIO, List<Camera> cameras, SwerveModuleIO frontLeft, SwerveModuleIO frontRight,
            SwerveModuleIO backLeft, SwerveModuleIO backRight, boolean isOpenLoop) {
        this.modules = new SwerveModule[] { new SwerveModule(frontLeft, FrontLeftModuleConstants.moduleID),
                new SwerveModule(frontRight, FrontRightModuleConstants.moduleID),
                new SwerveModule(backLeft, BackLeftModuleConstants.moduleID),
                new SwerveModule(backRight, BackRightModuleConstants.moduleID) };

        this.gyro = gyroIO;

        this.cameras = cameras;
        swerveOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyroInputs.yawPosition, getPositions(),
                new Pose2d());
        visionLessOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyroInputs.yawPosition,
                getPositions(), new Pose2d());
        setpointGenerator = new SwerveSetpointGenerator(DriveConstants.KINEMATICS, DriveConstants.MODULE_LOCATIONS);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
        var moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Logger.recordOutput("Unoptimized:", moduleStates);
        // swerveSetpoint = setpointGenerator.generateSetpoint(DriveConstants.MODULE_LIMITS,
        //     new SwerveSetpoint(getRobotRelativeSpeeds(), getStates()), ChassisSpeeds.discretize(chassisSpeeds, 0.02),
        //     SimConstants.LOOP_TIME); 
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_DRIVE_SPEED);
        setModuleStates(moduleStates);
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
            optimizedStates[module.getModuleID()] = module.setDesiredState(desiredStates[module.getModuleID()],
                    isOpenLoop);
        }
        swerveSetpoint.moduleStates = optimizedStates;
        Logger.recordOutput("SwerveStates/Desired", desiredStates);
        Logger.recordOutput("SwerveStates/Optimized", swerveSetpoint.moduleStates);
        Logger.recordOutput("SwerveStates/SetpointSpeeds", swerveSetpoint.chassisSpeeds);
        return desiredStates;
    }

    /**
     * Converts robot relative {@link ChassisSpeeds} to field relative.
     * 
     * @param robotRelativeSpeeds The robot relative speeds to convert
     * @return The field relative ChassisSpeeds.
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        Rotation2d angle = getPose().getRotation();
        return new ChassisSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond * angle.getCos()
                        - robotRelativeSpeeds.vyMetersPerSecond * angle.getSin(),
                robotRelativeSpeeds.vyMetersPerSecond * angle.getCos()
                        + robotRelativeSpeeds.vxMetersPerSecond * angle.getSin(),
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

        if (needGyroReset && !gyroInputs.isCalibrating) {
            gyro.zeroGyro();
            needGyroReset = false;
        }

        swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, getPositions());
        visionLessOdometry.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, getPositions());
        SmartDashboard.putBoolean("Has seen tag", hasSeenTag);
        for (Camera camera : cameras) {
            camera.periodic();
            Pose estimatedPose = camera.getEstimatedPose(getPose());
            if (estimatedPose.pose().isPresent()
                    && (hasSeenTag == false
                            || LobstahMath.getDistBetweenPoses(estimatedPose.pose().get().toPose2d(), getPose()) <= 8)
                    && Math.abs(estimatedPose.pose().get().getZ()) < 0.1 && (camera.getName().startsWith("front"))) {
                if (hasSeenTag == false) {
                    resetPose(new Pose2d(estimatedPose.pose().get().getX(), estimatedPose.pose().get().getY(),
                            getGyroAngle()));
                    hasSeenTag = true;
                }
                swerveOdometry.addVisionMeasurement(estimatedPose.pose().get().toPose2d(),
                        estimatedPose.timestamp().get(), estimatedPose.stdev().get());
                Logger.recordOutput("Vision/" + camera.getName() + "Used", true);
            } else
                Logger.recordOutput("Vision/" + camera.getName() + "Used", false);
        }
        resetPose(new Pose2d(MathUtil.clamp(getPose().getX(), 0, FieldConstants.FIELD_LENGTH),
                MathUtil.clamp(getPose().getY(), 0, FieldConstants.FIELD_WIDTH), getPose().getRotation()));
        field.setRobotPose(getPose());
        Logger.recordOutput("Odometry", getPose());
        Logger.recordOutput("Vision Less", visionLessOdometry.getEstimatedPosition());
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        // Log 3D odometry pose
        Pose3d robotPose3d = new Pose3d(getPose());
        robotPose3d = robotPose3d
                .exp(new Twist3d(0.0, 0.0,
                        Math.abs(gyroInputs.pitchPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0, 0.0,
                        gyroInputs.pitchPosition.getRadians(), 0.0))
                .exp(new Twist3d(0.0, 0.0,
                        Math.abs(gyroInputs.rollPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0,
                        gyroInputs.rollPosition.getRadians(), 0.0, 0.0));

        Logger.recordOutput("Odometry/Robot3d", robotPose3d);
        for (Camera camera : cameras) {
            Logger.recordOutput("Vision/" + camera.getName() + "/Pose3d", robotPose3d.plus(camera.getRobotToCamera()));
        }
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

        Logger.recordOutput("SwerveStates/Measured", getStates());
        SmartDashboard.putData("Drivebase subsystem", this);
    }

    /**
     * Constructs a command which holds this in place.
     * 
     * @return the constructed command
     */
    public Command stop() {
        return run(this::stopMotors);
    }

    /**
     * Constructs a command to auto-align to a particular pose. It will also light
     * the aligning LEDs at the beginning and turn them off at thend.
     * 
     * @param targetPoseSupplier a supplier which supplies the pose; the target pose
     *                           will be obtained from this supplier every time the
     *                           command is scheduled.
     * @return the constructed command
     */
    public Command alignToPose(Supplier<Pose2d> targetPoseSupplier) {
        return startRun(() -> {
            xController.reset();
            yController.reset();
            thetaController.reset();
            Pose2d targetPose = targetPoseSupplier.get();
            Logger.recordOutput("AutoAlignTargetPose", targetPose);
            xController.setSetpoint(targetPose.getX());
            yController.setSetpoint(targetPose.getY());
            thetaController.setSetpoint(targetPose.getRotation().getRadians());
        }, () -> {
            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(xController.calculate(getPose().getX()),
                    yController.calculate(getPose().getY()),
                    thetaController.calculate(getPose().getRotation().getRadians()), getPose().getRotation()));
        }).until(() -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint())
                .andThen(stop()).beforeStarting(() -> LEDs.getInstance().setAligning(true))
                .finallyDo(() -> LEDs.getInstance().setAligning(false));
    }

    /**
     * Constructs a command which aligns to the nearest scoring pose on the reef.
     * 
     * @param ccw Called every time the command is scheduled; If it returns true,
     *            get the nearest counter-clockwise scoring pose (B, D, et cetera);
     *            if false, get the nearest clockwise scoring pose (A, C, et
     *            cetera).
     * @return the constructed command
     */
    public Command alignToReef(BooleanSupplier ccw) {
        return alignToPose(() -> LobstahMath.getNearestScoringPose(getPose(), ccw.getAsBoolean()));
    }

    /**
     * Constructs a command which aligns to the nearest scoring pose on the reef.
     * 
     * @param ccw If true, get the nearest counter-clockwise scoring pose (B, D, et
     *            cetera); if false, get the nearest clockwise scoring pose (A, C,
     *            et cetera).
     * @return the constructed command
     */
    public Command alignToReef(boolean ccw) {
        return alignToReef(() -> ccw);
    }

    /**
     * Constructs a command which aligns to the alliance processor.
     * 
     * @return the constructed command
     */
    public Command alignToProcessor() {
        return alignToPose(() -> AlliancePoseMirror.mirrorPose2d(Poses.PROCESSOR));
    }

    /**
     * Constructs a command which aligns to the alliance barge.
     * 
     * @param ySupplier the supplier for the side-to-side (y axis) motion provided
     *                  by the user.
     * @return the constructed command
     */
    public Command alignToBarge(DoubleSupplier ySupplier) {
        return startRun(() -> {
            xController.reset();
            thetaController.reset();
            xController.setSetpoint(AlliancePoseMirror.isRedAlliance()
                    ? FieldConstants.FIELD_LENGTH - FieldConstants.Poses.BARGE_TRANSLATION_DEPTH_SETPOINT
                    : FieldConstants.Poses.BARGE_TRANSLATION_DEPTH_SETPOINT);
            thetaController.setSetpoint(AlliancePoseMirror.isRedAlliance() ? Rotation2d.kZero.getRadians()
                    : Rotation2d.k180deg.getRadians());
        }, () -> {
            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(xController.calculate(getPose().getX()),
                    AlliancePoseMirror.isRedAlliance() ? -ySupplier.getAsDouble() : ySupplier.getAsDouble(),
                    thetaController.calculate(getPose().getRotation().getRadians()), getPose().getRotation()));
        }).until(() -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint())
                .andThen(stop()).beforeStarting(() -> LEDs.getInstance().setAligning(true))
                .finallyDo(() -> LEDs.getInstance().setAligning(false));
    }

    /**
     * Create a new command to drive field-relative.
     * 
     * @param strafeXSupplier  Supplier for strafe in X direction, e.g. from a
     *                         joystick.
     * @param strafeYSupplier  Supplier for strafe in Y direction, e.g. from a
     *                         joystick.
     * @param rotationSupplier Supplier for rotation, e.g. from a joystick.
     * @return constructed command
     */
    public Command joystickDrive(DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier,
            DoubleSupplier rotationSupplier) {
        return run(() -> {
            double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble()),
                    IOConstants.JOYSTICK_DEADBAND);
            Rotation2d linearDirection = linearMagnitude > 0
                    ? new Rotation2d(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble())
                    : Rotation2d.kZero;
            if (AlliancePoseMirror.isRedAlliance()) linearDirection = linearDirection.plus(Rotation2d.k180deg);
            double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), IOConstants.JOYSTICK_DEADBAND);

            // Square values
            if (ControllerIOConstants.SQUARE_INPUTS) {
                linearMagnitude = linearMagnitude * linearMagnitude;
                omega = Math.copySign(omega * omega, omega);
            }

            // Calculate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.MAX_DRIVE_SPEED,
                    linearVelocity.getY() * DriveConstants.MAX_DRIVE_SPEED, omega * DriveConstants.MAX_ANGULAR_SPEED);

            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getPose().getRotation()));
        }).finallyDo(this::stopMotors);
    }

    /**
     * Drive the robot robot-relative.
     * 
     * @param strafeX  constant strafe in X (m/s)
     * @param strafeY  constant strafe in Y (m/s)
     * @param rotation constant rotation (rad/s)
     * @return constructed command
     */
    public Command relativeDrive(double strafeX, double strafeY, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(strafeX, strafeY, rotation);
        return run(() -> {
            driveRobotRelative(speeds);
        }).finallyDo(this::stopMotors);
    }
}
