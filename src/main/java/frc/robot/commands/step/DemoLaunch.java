// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.LightStrip.Color;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.Pose2dUtil;

public class DemoLaunch extends BaseLaunch {
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private double m_initialLiftHeightMeters = 0;
  private Rotation2d m_lastLauncherTilt = null;
	private Timer m_focusTimer = new Timer();
  private boolean m_focusTimerStarted = false;
  private LightStrip m_lightStrip;
  private Supplier<Pose3d> m_getDemoTargetPose;

  /** Creates a new Lanch Partay (DEMO Mode). */
  public DemoLaunch(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake,
      LightStrip lightStrip,
      Supplier<Pose3d> getDemoTargetPose,
      Supplier<LauncherTarget> getDefaultLauncherTarget) {
    super(lift, launcher, feeder, intake, getDefaultLauncherTarget);
    m_swerveDrive = swerveDrive;
    m_driver = driver;
    m_lightStrip = lightStrip;
    m_getDemoTargetPose = getDemoTargetPose;

    addRequirements(lift, launcher, feeder, swerveDrive, lightStrip);
  }

  @Override
  public void initialize() {
    m_lastLauncherTilt = m_launcher.getEncoderTiltAngle();
    m_initialLiftHeightMeters = m_lift.getCurrentHeightMeters();
    m_swerveDrive.resetPids();
    m_focusTimer.stop();
    m_focusTimer.reset();
    m_focusTimer.start();
    m_focusTimerStarted = false;
    m_lightStrip.setSingleColor(Color.WHITE);
    super.initialize();
  }

  @Override
  public void execute() {
    var targetAngle = getTargetAngle();
    m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 1.0);
    super.execute();
  }

  private Rotation2d getTargetAngle() {
    Translation2d targetLocation = m_getDemoTargetPose.get().toPose2d().getTranslation();
    Translation2d distance = targetLocation.minus(m_swerveDrive.getPose().getTranslation());
    Rotation2d targetAngle = distance.getAngle();
    return targetAngle;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
    super.end(interrupted);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    var targetPose = m_getDemoTargetPose.get();
    var targetHeightMeters = targetPose.getZ();
    var targetDistanceMeters = Pose2dUtil.getDistanceMeters(targetPose.toPose2d(), m_swerveDrive.getPose());
    var target = LauncherModel.getLauncherTarget(targetHeightMeters, m_initialLiftHeightMeters, targetDistanceMeters, m_lastLauncherTilt, TargetAngleMode.Lower);
    m_lastLauncherTilt = target.isPresent() ? target.get().getTiltAngle() : m_lastLauncherTilt;
    return target;
  }

  @Override
  protected boolean isClearToLaunch() {
    var isOdometryInRange = Math.abs(getDriveAngleErrorDegrees()) < VisionConstants.TxLaunchTolerance;
    if (isOdometryInRange && !m_focusTimerStarted) {
      m_focusTimer.start();
      m_focusTimerStarted = true;
    }
    var hasTimedOut = m_focusTimer.hasElapsed(0.5);
    if (m_focusTimerStarted) {
      m_lightStrip.setSingleColor(Color.PURPLE);
    }
    return hasTimedOut && isOdometryInRange;
  }

  private double getDriveAngleErrorDegrees() {
    return getTargetAngle().minus(m_swerveDrive.getOdometryRotation()).getDegrees();
  }

  @Override
  protected List<String> getLaunchErrors() {
    var errors = super.getLaunchErrors();
    errors.add(formatError("Swerve Angle Degrees", getDriveAngleErrorDegrees()));
    return errors;
  }
}
