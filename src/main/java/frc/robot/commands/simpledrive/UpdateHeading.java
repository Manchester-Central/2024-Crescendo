// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.simpledrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DriveDirection;

/**
 * A command to update the robot's heading, with respect to the current alliance (0 degrees for blue means 180 degrees for red)
 */
public class UpdateHeading extends InstantCommand {
  private final SwerveDrive m_swerveDrive;
  private final DriveDirection m_direction;

  public UpdateHeading(SwerveDrive swerveDrive, DriveDirection direction) {
    m_swerveDrive = swerveDrive;
    m_direction = direction;
  }

  @Override
  public void initialize() {
      m_swerveDrive.resetHeading(m_direction.getAllianceAngle());
  }
}
