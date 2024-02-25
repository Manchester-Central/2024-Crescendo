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
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.util.FieldPose2024;

public class FireIntoAmp extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private BaseSwerveDrive m_swerveDrive;
  private Vision m_vision;
  private ampState state = ampState.vision;
  private Pose2d m_finalPose;

  /** Creates a new FireIntoAmp. */
  public FireIntoAmp(Lift lift, Launcher launcher, BaseSwerveDrive swerveDrive, Vision vision) {
    m_lift = lift;
    m_launcher = launcher;
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    addRequirements(lift, launcher, swerveDrive);

  }

  public enum ampState {
    moveIntoPosition, vision, fireIntoAmp
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double x = Constants.robotLengthMeters/2 * Math.cos(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
    double y = Constants.robotLengthMeters/2 * Math.sin(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
    Translation2d pose = FieldPose2024.Amp.getCurrentAlliancePose().getTranslation().plus(new Translation2d(x, y));
    m_swerveDrive.setTarget(new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation()));
    m_finalPose = new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == ampState.moveIntoPosition) {
      if (m_swerveDrive.atTarget(/* 1.5 */) && m_lift.atTargetHeight(LiftConstants.DefaultAmpMeters)
          && m_launcher.atTargetAngle(Rotation2d.fromDegrees(10))) {
        m_swerveDrive.stop();
        state = ampState.vision;
      } else {
        m_swerveDrive.moveToTarget(0.40);
        
      }

    } else if(state == ampState.vision) { 

      Pose2d visionPose = m_vision.getPose();
      Translation2d translationErrorMeters = visionPose.getTranslation().minus(m_finalPose.getTranslation());
      Rotation2d rotationError = visionPose.getRotation().minus(m_finalPose.getRotation());
      m_lift.moveToHeight(LiftConstants.AmpHeight);
        m_launcher.setTiltAngle((LauncherConstants.AmpAngle));
      

      if (translationErrorMeters.getNorm() < 0.01 && rotationError.getDegrees() < 2 
          && m_lift.atTargetHeight(LiftConstants.AmpHeight) && m_launcher.atTargetAngle((LauncherConstants.AmpAngle))) {
        state = ampState.fireIntoAmp; 
        m_swerveDrive.stop();

      } else {
        var robotPose = m_swerveDrive.getPose();
        m_swerveDrive.setTarget(new Pose2d(translationErrorMeters.plus(robotPose.getTranslation()), rotationError.plus(robotPose.getRotation())));
        m_swerveDrive.moveToTarget(0.15);
      }
        

    }  else {
      m_launcher.setLauncherPower(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state = ampState.vision;
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
