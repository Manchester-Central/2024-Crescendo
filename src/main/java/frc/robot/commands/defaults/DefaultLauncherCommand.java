// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;

public class DefaultLauncherCommand extends Command {
  private Launcher m_launcher;
  private Gamepad m_operator;
  public static double MaxTiltSpeed = 0.08;

  private Supplier<LauncherTarget> m_getDefaultLauncherTarget;
  private Supplier<Boolean> m_hasNoteInFeeder;

  /** Creates a new DefaultLauncherCommand. */
  public DefaultLauncherCommand(Launcher launcher, Gamepad operator,  Supplier<LauncherTarget> getDefaultLauncherTarget, Supplier<Boolean> hasNoteInFeeder) {
    m_launcher = launcher;
    m_operator = operator;
    m_hasNoteInFeeder = hasNoteInFeeder;
    addRequirements(launcher);
    m_getDefaultLauncherTarget = getDefaultLauncherTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcher.setTiltSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_launcher.setTiltSpeed(m_operator.getLeftY() * MaxTiltSpeed);
    if (RobotContainer.PreSpinEnabled) {
      var launcherTarget = m_getDefaultLauncherTarget.get();
      m_launcher.setTiltAngle(launcherTarget.getTiltAngle());
      m_launcher.setLauncherRPM(launcherTarget.getLeftLauncherSpeedRPM(), launcherTarget.getRightLauncherSpeedRPM());
    } else {
      m_launcher.setLauncherPower(0.0);
      m_launcher.setTiltSpeed(m_operator.getLeftY() * MaxTiltSpeed);
    }
    // m_launcher.setTiltAngle(Rotation2d.fromDegrees(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
