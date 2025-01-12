// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;

public class SwerveDriveStopCommand extends Command {
  /** Creates a new SwerveDriveStopCommand. */
  private final DriveBase driveBase;
  public SwerveDriveStopCommand(DriveBase driveBase) {
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    driveBase.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
