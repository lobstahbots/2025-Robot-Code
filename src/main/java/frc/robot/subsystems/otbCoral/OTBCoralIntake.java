// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.otbCoral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OTBCoralConstants;

public class OTBCoralIntake extends SubsystemBase {
    
    //joint 1 refers to the lower joint the one directly attached to the drive base while joint 2 refers to the one higher the 2nd DOF driverMotor refers to the motor that spins the intake
    private final SparkMax joint1Motor;
    private final SparkMax joint2Motor;
    private final SparkMax driverMotor;

    private final DutyCycleEncoder joint1Encoder;
    private final DutyCycleEncoder joint2Encoder;

    private final ProfiledPIDController joint1PosPID = new ProfiledPIDController(OTBCoralConstants.JOINT_1_kP, OTBCoralConstants.JOINT_1_kI, OTBCoralConstants.JOINT_1_kD);
    private final ArmFeedforward joint1Feedforward;

    public OTBCoralIntake(int joint1MotorID, int joint2MotorID, int intakeMotorID) {
        this.joint1Motor = new SparkMax(joint1MotorID, MotorType.kBrushless);
        this.joint2Motor = new SparkMax(joint2MotorID, MotorType.kBrushless);
        this.driverMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);

        this.joint1Encoder = new DutyCycleEncoder(OTBCoralConstants.JOINT_1_ENCODER_CHANNEL);
        this.joint2Encoder = new DutyCycleEncoder(OTBCoralConstants.JOINT_2_ENCODER_CHANNEL);
        
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(OTBCoralConstants.JOINT_MOTOR_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(false);

        joint1Motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        joint2Motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig driverConfig = new SparkMaxConfig();
        driverConfig.smartCurrentLimit(OTBCoralConstants.DRIVER_MOTOR_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(false);

        driverMotor.configure(driverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
