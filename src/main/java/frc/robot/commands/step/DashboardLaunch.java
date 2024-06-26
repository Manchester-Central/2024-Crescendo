// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;

// TODO: Implement actual control logic
public class DashboardLaunch extends BaseLaunch {

  double m_targetRPM = 5000;
  double m_speedOffsetRPM = 0;
  double m_targetHeight = 0.096;
  
  /** Creates a new Lanch Partay. */
  public DashboardLaunch(Lift lift, Launcher launcher, Feeder feeder, Intake intake, Supplier<LauncherTarget> getDefaultLauncherTarget) {
    super (lift, launcher, feeder, intake, getDefaultLauncherTarget);
    SmartDashboard.putNumber("DSLaunch Target RPM", m_targetRPM);
    SmartDashboard.putNumber("DSLaunch Speed Offset RPM", m_speedOffsetRPM);
    SmartDashboard.putNumber("DSLaunch Target Height Meters", m_targetHeight);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    m_targetRPM = SmartDashboard.getNumber("DSLaunch Target RPM", m_targetRPM);
    m_speedOffsetRPM = SmartDashboard.getNumber("DSLaunch Speed Offset RPM", m_speedOffsetRPM);
    m_targetHeight = SmartDashboard.getNumber("DSLaunch Target Height Meters", m_targetHeight);
    return Optional.of(new LauncherTarget(m_targetRPM, m_speedOffsetRPM,m_launcher.getEncoderTiltAngle().getDegrees(), m_targetHeight));
  }

  @Override
  protected boolean isClearToLaunch() {
      return true;
  }
}
