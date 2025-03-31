// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final int moduleID;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModuleIO io;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.KS,
            SwerveConstants.KV, SwerveConstants.KA);
    private final PIDController driveController;
    private final PIDController angleController;

    /**
     * Constructs a SwerveModule. The SwerveModule class contains methods relating
     * to both real and simulated modules, whereas SwerveModuleReal and
     * SwerveModuleSim are implementations of SwerveModule IO that contain methods
     * specific to a case.
     * 
     * @param io       the {@link SwerveModuleIO} that handles loggable fields as
     * @param moduleID The module ID number. ID numbers range from 0-3: FrontLeft,
     *                 BackLeft, FrontRight, BackRight.
     */
    public SwerveModule(SwerveModuleIO io, int moduleID) {
        this.io = io;
        this.moduleID = moduleID;
        this.driveController = new PIDController(SwerveConstants.DRIVE_PID_P, SwerveConstants.DRIVE_PID_I,
                SwerveConstants.DRIVE_PID_D);
        this.angleController = new PIDController(SwerveConstants.TURN_PID_P, SwerveConstants.TURN_PID_I,
                SwerveConstants.TURN_PID_D);
        angleController.enableContinuousInput(SwerveConstants.TURN_PID_MIN_INPUT, SwerveConstants.TURN_PID_MAX_INPUT);
    }

    /**
     * Sets voltage of drive motor and holds angle at 0 for use during
     * characterization ramp routines.
     * 
     * @param voltage The voltage to set the drive motor to.
     * @see CharacterizableSubsystem
     */
    public void runVolts(double voltage) {
        io.setDriveVoltage(voltage);
        io.setTurnVoltage(angleController.calculate(getAngle().getRadians(), 0));
    }

    /** Sets the voltages of both motors to 0 */
    public void stop() {
        io.setDriveVoltage(0);
        io.setTurnVoltage(0);
    }

    /**
     * Returns the module ID.
     *
     * @return The ID number of the module (0-3).
     */
    public int getModuleID() {
        return moduleID;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(moduleID), inputs);
        io.periodic();
    }

    /**
     * Sets the desired state for the module. Optimizes state.
     *
     * @param desiredState A {@link SwerveModuleState} with desired speed and angle.
     * @return The optimized SwerveModuleState.
     */
    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(inputs.turnAbsolutePosition);
        io.setTurnVoltage(angleController.calculate(getAngle().getRadians(), desiredState.angle.getRadians()));

        // Update velocity based on turn error
        desiredState.speedMetersPerSecond *= Math.cos(angleController.getError());

        // Run drive controller
        double velocityRadPerSec = desiredState.speedMetersPerSecond / (RobotConstants.WHEEL_DIAMETER / 2);
        io.setDriveVoltage(feedforward.calculate(velocityRadPerSec)
                + driveController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

        return desiredState;
    }

    /**
     * Sets whether brake mode is enabled.
     * 
     * @param mode New {@link IdleMode}, applied to both drive and turn motors.
     */
    public void setIdleMode(IdleMode mode) {
        io.setDriveIdleMode(mode);
        io.setTurnIdleMode(mode);
    }

    /**
     * @return The current turn angle of the module.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnAbsolutePosition.getRadians());
    }

    /**
     * @returns The current turn angle of the module.
     */
    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(inputs.turnAbsolutePosition.getRadians());
    }

    /**
     * @returns The current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePosition.getRotations() * Math.PI * RobotConstants.WHEEL_DIAMETER;
    }

    /**
     * @returns The current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * RobotConstants.WHEEL_DIAMETER / 2;
    }

    /**
     * @returns The module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAbsoluteAngle());
    }

    /**
     * @returns The module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAbsoluteAngle());
    }
}