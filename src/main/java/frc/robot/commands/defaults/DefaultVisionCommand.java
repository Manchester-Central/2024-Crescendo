// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.vision.CameraInterface.CameraMode;
import frc.robot.util.FieldPose2024;

public class DefaultVisionCommand extends Command {

  private Vision m_vision;

  /** Creates a new DefaultVisionCommand. */
  public DefaultVisionCommand(Vision vision) {
    m_vision = vision;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var frontCamera = m_vision.getCamera(CameraDirection.Front);
    if (DriverStation.isTest()) {
      frontCamera.setMode(CameraMode.DEMO);
      frontCamera.setPriorityID(15);
      return;
    }
    // We are only using the front limelight for speaker tracking, so let's always make sure it's using that for tx/ty tracking
    var priorityID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    if(frontCamera.getPriorityID() != priorityID) {
      frontCamera.setPriorityID(priorityID);
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
