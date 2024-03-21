// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.FieldPose2024;

public class LaunchSetDistance extends BaseLaunch {
  private double m_initialLiftHeightMeters = 0;
  private Rotation2d m_lastLauncherTilt = null;
  private FieldPose2024 m_launchPose;
  /** Creates a new Lanch Partay. */
  public LaunchSetDistance(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      Intake intake,
      FieldPose2024 launchPose,
      Supplier<LauncherSpeeds> getDefaultLauncherSpeeds

  ) {
    super(lift, launcher, feeder, intake, getDefaultLauncherSpeeds);
    m_launchPose = launchPose;
    addRequirements(lift, launcher, feeder,intake);
  }

  @Override
  public void initialize() {
    m_lastLauncherTilt = m_launcher.getAbsoluteTiltAngle();
    m_initialLiftHeightMeters = m_lift.getCurrentHeightMeters();
    super.initialize();
  }
  


  @Override
  protected Optional<LauncherTarget> getTargets() {
    double distanceToSpeakerMeters = m_launchPose.distanceTo(FieldPose2024.Speaker.getCurrentAlliancePose());
    var target = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_initialLiftHeightMeters, distanceToSpeakerMeters, m_lastLauncherTilt, TargetAngleMode.Lower);
    m_lastLauncherTilt = target.isPresent() ? target.get().getTiltAngle() : m_lastLauncherTilt;
    return target;
  }

  @Override
  protected boolean isClearToLaunch() {
    return true;
  }
}
