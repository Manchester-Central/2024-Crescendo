// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;

public class DefaultVisionCommand extends Command {

  private Vision m_vision;
  private BaseSwerveDrive m_swerveDrive;

  /** Creates a new DefaultVisionCommand. */
  public DefaultVisionCommand(Vision vision, BaseSwerveDrive swerveDrive) {
    m_vision = vision;
    m_swerveDrive = swerveDrive;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (VisionConstants.UseVisionForOdometry && m_vision.getPose() != null) {
      m_swerveDrive.addVisionMeasurement(m_vision.getPose(), m_vision.getLatencySeconds());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
