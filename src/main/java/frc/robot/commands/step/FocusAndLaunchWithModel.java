// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.FlywheelTable;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.AngleUtil;
import frc.robot.util.FieldPose2024;

public class FocusAndLaunchWithModel extends BaseLaunch {
  private Vision m_vision;
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private boolean m_beenAboveThreshold = false;

  /** Creates a new Lanch Partay. */
  public FocusAndLaunchWithModel(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      Vision vision,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake
  ) {
    super(lift, launcher, feeder, intake);
    m_vision = vision;
    m_swerveDrive = swerveDrive;
    m_driver = driver;

    addRequirements(lift, launcher, feeder, vision, swerveDrive);
  }

  @Override
  public void initialize() {
    m_beenAboveThreshold = false;
    m_swerveDrive.resetPids();
    //m_vision.getCamera(CameraDirection.front).setMode(m_vision.getSpeakerTrackingMode());
    super.initialize();
  }

  @Override
  public void execute() {
    if (m_vision.getCamera(CameraDirection.front).hasTarget()) {
      var currentPose = m_swerveDrive.getPose();
      var currentRotation = currentPose.getRotation();
      var rotation = AngleUtil.GetEstimatedAngleToGoal(m_vision, currentPose, currentRotation);
      m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), rotation, 1.0);
    } else {
      m_swerveDrive.moveFieldRelative(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), -m_driver.getSlewRightX());
    }
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
    super.end(interrupted);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    var ty = m_vision.getCamera(CameraDirection.front).getTargetElevation(true);
    if (!m_vision.getCamera(CameraDirection.front).hasTarget()) {
      return Optional.empty();
    }
    double distanceToSpeakerMeters = 0; // TO-DO: Convert ty to distance.
    var targets = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_lift.getCurrentHeightMeters(), distanceToSpeakerMeters);
    // var targets = m_flywheelTableLowerHeight.getIdealTarget(ty);
    SmartDashboard.putString("launch targets", targets.toString());
    return Optional.ofNullable(targets);
  }

  @Override
  protected boolean isClearToLaunch() {
    // TODO - handle logic better for when shooting on the fly
    return Math.abs(m_vision.getCamera(CameraDirection.front).getTargetAzimuth(true)) < VisionConstants.TxLaunchTolerance;
  }
}
