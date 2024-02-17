// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.PortUnreachableException;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.util.FieldPose2024;

public class FireIntoAmp extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private BaseSwerveDrive m_swerveDrive;
  private ampState state = ampState.moveIntoPosition;

  /** Creates a new FireIntoAmp. */
  public FireIntoAmp(Lift lift, Launcher launcher, BaseSwerveDrive swerveDrive) {
    m_lift = lift;
    m_launcher = launcher;
    m_swerveDrive = swerveDrive;
    addRequirements(lift, launcher, swerveDrive);

  }

  public enum ampState {
    moveIntoPosition, fireIntoAmp
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double x = 0.7 * Math.cos(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
    double y = 0.7 * Math.sin(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
    Translation2d pose = FieldPose2024.Amp.getCurrentAlliancePose().getTranslation().plus(new Translation2d(x, y));
    m_swerveDrive.setTarget(new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == ampState.moveIntoPosition) {
      if (m_swerveDrive.atTarget() && m_lift.atTargetHeight(1)
          && m_launcher.atTargetAngle(Rotation2d.fromDegrees(10))) {
        m_swerveDrive.stop();
        state = ampState.fireIntoAmp;
      } else {
        m_swerveDrive.moveToTarget(0.40);
        m_lift.moveToHeight(1);
        m_launcher.setLauncherAngle(Rotation2d.fromDegrees(10));
      }

    } else {
      m_launcher.setLauncherPower(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state = ampState.moveIntoPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == ampState.fireIntoAmp) {
      return true;
      // TODO Check if note is gone
    } else {
      return false;
    }
  }
}
