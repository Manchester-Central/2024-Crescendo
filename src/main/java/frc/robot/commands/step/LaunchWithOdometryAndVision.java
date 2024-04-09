// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LightStrip.Color;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.util.AngleUtil;
import frc.robot.util.FieldPose2024;

public class LaunchWithOdometryAndVision extends BaseLaunch {
  private BaseSwerveDrive m_swerveDrive;
  private CameraInterface m_camera;
  private Gamepad m_driver;
  private double m_initialLiftHeightMeters = 0;
  private Rotation2d m_lastLauncherTilt = null;
  private Supplier<Boolean> m_isVisionEnabledSupplier;
	private Timer m_focusTimer = new Timer();
  private LightStrip m_lightStrip;

  /** Creates a new Lanch Partay. */
  public LaunchWithOdometryAndVision(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake,
      Vision vision,
      LightStrip lightStrip,
      Supplier<LauncherTarget> getDefaultLauncherTarget,
      Supplier<Boolean> isVisionEnabledSupplier) {
    super(lift, launcher, feeder, intake, getDefaultLauncherTarget);
    m_swerveDrive = swerveDrive;
    m_driver = driver;
    m_camera = vision.getCamera(CameraDirection.Front);
    m_isVisionEnabledSupplier = isVisionEnabledSupplier;
    m_lightStrip = lightStrip;

    addRequirements(lift, launcher, feeder, swerveDrive, lightStrip);
  }

  @Override
  public void initialize() {
    m_lastLauncherTilt = m_launcher.getEncoderTiltAngle();
    m_initialLiftHeightMeters = m_lift.getCurrentHeightMeters();
    m_swerveDrive.resetPids();
    m_camera.setPriorityID(DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4);
    m_focusTimer.stop();
    m_focusTimer.reset();
    m_focusTimer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    var targetAngle = getTargetAngle();
    m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 1.0);
    super.execute();
  }

  private Rotation2d getTargetAngle() {
    // Aim with april tags if we have vision, otherwise use odometry
    var isVisionEnabled = m_isVisionEnabledSupplier.get();
    if (isVisionEnabled && m_camera.hasTarget()) {
      var currentPose = m_swerveDrive.getPose();
      return AngleUtil.GetEstimatedAngleToGoal(m_camera, currentPose, currentPose.getRotation());
    }
    Translation2d speakerLocation = FieldPose2024.Speaker.getCurrentAlliancePose().getTranslation();
    Translation2d distance = speakerLocation.minus(m_swerveDrive.getPose().getTranslation());
    Rotation2d targetAngle = distance.getAngle();
    return targetAngle;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
    // m_vision.getCamera(CameraDirection.Front).resetPriorityID();
    super.end(interrupted);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    // Our distance to the speaker with odemetry seems reliable, so use that instead of vision
    double distanceToSpeakerMeters = FieldPose2024.Speaker.getDistanceFromLocation(m_swerveDrive.getPose());
    var target = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_initialLiftHeightMeters, distanceToSpeakerMeters, m_lastLauncherTilt, TargetAngleMode.Lower);
    m_lastLauncherTilt = target.isPresent() ? target.get().getTiltAngle() : m_lastLauncherTilt;
    return target;
  }

  @Override
  protected boolean isClearToLaunch() {
    var isVisionEnabled = m_isVisionEnabledSupplier.get();
    if (isVisionEnabled && m_camera.hasTarget()) {
      return Math.abs(m_camera.getTargetAzimuth(true)) < VisionConstants.TxLaunchTolerance;
    }
    var hasTimedOut = m_focusTimer.hasElapsed(1.7);
    if (hasTimedOut) {
      m_lightStrip.setSingleColor(Color.PURPLE);
    }
    return hasTimedOut && Math.abs(getDriveAngleErrorDegrees()) < VisionConstants.TxLaunchTolerance;
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
