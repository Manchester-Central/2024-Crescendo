// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.FlywheelTable;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.AngleUtil;

public class FocusAndLaunch extends BaseLaunch {
  private FlywheelTable m_flywheelTableLowerHeight;
  private FlywheelTable m_flywheelTableUpperHeight;
  private Vision m_vision;
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private boolean m_beenAboveThreshold = false;

  /** Creates a new Lanch Partay. */
  public FocusAndLaunch(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      FlywheelTable flywheelTableLowerHeight,
      FlywheelTable flywheelTableUpperHeight,
      Vision vision,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake
  ) {
    super(lift, launcher, feeder, intake);
    m_flywheelTableLowerHeight = flywheelTableLowerHeight;
    m_flywheelTableUpperHeight = flywheelTableUpperHeight;
    m_vision = vision;
    m_swerveDrive = swerveDrive;
    m_driver = driver;

    addRequirements(lift, launcher, feeder, vision, swerveDrive);
  }

  @Override
  public void initialize() {
    m_beenAboveThreshold = false;
    m_swerveDrive.resetPids();
    m_vision.getCamera(CameraDirection.front).setPriorityID(DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4);
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
    m_vision.getCamera(CameraDirection.front).resetPriorityID();
    super.end(interrupted);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    var ty = m_vision.getCamera(CameraDirection.front).getTargetElevation(true);
    if (!m_vision.getCamera(CameraDirection.front).hasTarget()) {
      return Optional.empty();
    }
    if (ty <= m_flywheelTableLowerHeight.getMinTY()) {
      m_beenAboveThreshold = true;
    }
    var targets = (m_beenAboveThreshold ? m_flywheelTableUpperHeight : m_flywheelTableLowerHeight).getIdealTargetByTY(ty);
    return targets;
  }

  @Override
  protected boolean isClearToLaunch() {
    // TODO - handle logic better for when shooting on the fly
    return Math.abs(m_vision.getCamera(CameraDirection.front).getTargetAzimuth(true)) < VisionConstants.TxLaunchTolerance;
  }
}
