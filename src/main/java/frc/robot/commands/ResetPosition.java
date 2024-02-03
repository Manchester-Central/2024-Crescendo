// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoUtil;


public class ResetPosition extends Command {
  private BaseSwerveDrive m_swerveDrive;
  private Pose2d m_pose;
  /** Creates a new resetPosition. */
  public ResetPosition(Pose2d pose, BaseSwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.resetPose(m_pose);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, BaseSwerveDrive swerveDrive) {
		// double translationTolerance = AutoUtil.getTranslationTolerance(parsedCommand);
		// double maxPercentSpeed = AutoUtil.getMaxPercentSpeed(parsedCommand);
		Pose2d pose = AutoUtil.getDrivePose(parsedCommand);
		if(pose == null) {
			return new InstantCommand();
		}
		return new ResetPosition(pose, swerveDrive);
	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
