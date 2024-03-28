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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.AngleUtil;
import frc.robot.util.FieldPose2024;

public class LaunchWithOdometry extends BaseLaunch {
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private double m_initialLiftHeightMeters = 0;
  private Rotation2d m_lastLauncherTilt = null;

  /** Creates a new Lanch Partay. */
  public LaunchWithOdometry(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake,
      Supplier<LauncherSpeeds> getDefaultLauncherSpeeds
  ) {
    super(lift, launcher, feeder, intake, getDefaultLauncherSpeeds);
    m_swerveDrive = swerveDrive;
    m_driver = driver;

    addRequirements(lift, launcher, feeder, swerveDrive);
  }

  @Override
  public void initialize() {
    m_lastLauncherTilt = m_launcher.getAbsoluteTiltAngle();
    m_initialLiftHeightMeters = m_lift.getCurrentHeightMeters();
    m_swerveDrive.resetPids();
    super.initialize();
  }

  @Override
  public void execute() {
    var targetAngle = getTargetAngle();
    m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 1.0);
    super.execute();
  }

  private Rotation2d getTargetAngle() {
    Translation2d speakerLocation = FieldPose2024.Speaker.getCurrentAlliancePose().getTranslation();
    Translation2d distance = speakerLocation.minus(m_swerveDrive.getPose().getTranslation());
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
    double distanceToSpeakerMeters = FieldPose2024.Speaker.getDistanceFromLocation(m_swerveDrive.getPose());
    var target = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_initialLiftHeightMeters, distanceToSpeakerMeters, m_lastLauncherTilt, TargetAngleMode.Lower);
    m_lastLauncherTilt = target.isPresent() ? target.get().getTiltAngle() : m_lastLauncherTilt;
    return target;
  }

  @Override
  protected boolean isClearToLaunch() {
    // TODO - handle logic better for when shooting on the fly
    return Math.abs(getDriveAngleErrorDegrees()) < VisionConstants.TxLaunchTolerance;
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
