// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.util.DashboardNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.FieldPose2024;

public class LobOntoFieldSetDistance extends BaseLaunch {
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private FieldPose2024 m_aimPose;
  private double m_distanceMeters;
  private double m_liftHeight;
  private Rotation2d m_launchAngle;
  private boolean m_isSourceIntake;

  /** Creates a new Lanch Partay. */
  public LobOntoFieldSetDistance(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake,
      FieldPose2024 aimPose,
      double distanceMeters,
      double liftHeight,
      Rotation2d launchAngle,
      Supplier<LauncherTarget> getDefaultLauncherTarget,
      boolean isSourceIntake
  ) {
    super(lift, launcher, feeder, intake, getDefaultLauncherTarget);
    m_swerveDrive = swerveDrive;
    m_driver = driver;
    m_aimPose = aimPose;
    m_distanceMeters = distanceMeters;
    m_liftHeight = liftHeight;
    m_launchAngle = launchAngle;
    m_isSourceIntake = isSourceIntake;

    addRequirements(lift, launcher, feeder, swerveDrive);
  }

  @Override
  protected double getFeederLaunchSpeed() {
    return 1.0;
  }

  @Override
  protected double getTrapLaunchSpeed() {
    return m_isSourceIntake ? -1.0 : 1.0;
  }

  @Override
  public void initialize() {
    m_swerveDrive.resetPids();
    super.initialize();
  }

  @Override
  public void execute() {
    var targetAngle = m_aimPose.getCurrentAlliancePose().getRotation();
    m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 1.0);
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
    return LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_liftHeight, m_distanceMeters, m_launchAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  protected boolean isClearToLaunch() {
    return true;
  }

}
