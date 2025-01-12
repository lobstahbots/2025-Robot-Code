package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = initialStates;
    }
}