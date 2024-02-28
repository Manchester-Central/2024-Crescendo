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
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TableData;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
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
      Gamepad driver
  ) {
    super(lift, launcher, feeder);
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
    m_vision.setMode(m_vision.getSpeakerTrackingMode());
    super.initialize();
  }

  @Override
  public void execute() {
    if (m_vision.hasTarget()) {
      var currentPose = m_swerveDrive.getPose();
      var currentRotation = currentPose.getRotation();
      var rotation = AngleUtil.GetEstimatedAngleToGoal(m_vision, currentPose, currentRotation);
      m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), rotation, 1.0);
    } else {
      m_swerveDrive.moveFieldRelative(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), -m_driver.getSlewRightX());
    }
    super.execute();
  }

  public static Command createAutoCommand(
      ParsedCommand parsedCommand,
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      FlywheelTable flywheelTableLowerHeight,
      FlywheelTable flywheelTableUpperHeight,
      Vision vision,
      SwerveDrive swerveDrive,
      Gamepad driver
    ) {
      return new FocusAndLaunch(lift, launcher, feeder, flywheelTableLowerHeight, flywheelTableUpperHeight, vision,
      swerveDrive, driver);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
    m_vision.updateAprilTagMode(m_swerveDrive.getPose());
    super.end(interrupted);
  }

  @Override
  protected Optional<TableData> getTargets() {
    // var pose = m_vision.getPose();
    // if (pose == null) {
    // return Optional.empty();
    // }
    // var distanceMeters = FieldPose2024.Speaker.distanceTo(pose);
    // if (distanceMeters >= m_flywheelTableLowerHeight.getMaxDistance()) {
    // m_beenAboveThreshold = true;
    // }
    // return (m_beenAboveThreshold ? m_flywheelTableUpperHeight :
    // m_flywheelTableLowerHeight).getIdealTarget(distanceMeters);
    var ty = m_vision.getYAngle();
    if (!m_vision.hasTarget()) {
      return Optional.empty();
    }
    if (ty <= m_flywheelTableLowerHeight.getMinDistance()) {
      m_beenAboveThreshold = true;
    }
    // var targets = (m_beenAboveThreshold ? m_flywheelTableUpperHeight :
    // m_flywheelTableLowerHeight).getIdealTarget(ty);
    var targets = m_flywheelTableLowerHeight.getIdealTarget(ty);
    SmartDashboard.putString("launch targets", targets.toString());
    return targets;
  }

  @Override
  protected boolean isClearToLaunch() {
    // TODO - handle logic better for when shooting on the fly
    return Math.abs(m_vision.getXAngle()) < VisionConstants.TxLaunchTolerance;
  }
}