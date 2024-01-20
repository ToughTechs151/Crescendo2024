// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends InstantCommand {

  private final DriveSubsystem drive;

  public ResetOdometry(DriveSubsystem drive) {

    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(
        new Pose2d(
            Constants.DriveConstants.startX,
            Constants.DriveConstants.startY,
            new Rotation2d(DriveConstants.startHeading)));
  }
}
