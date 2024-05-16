// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.RumbleManager;

// TODO: Implement actual control logic
public class RunIntakeUntilNote extends RunIntake {

  /** Creates a new RunIntake. */
  public RunIntakeUntilNote(Intake intake, Lift lift, Feeder feeder, Launcher launcher, Supplier<LauncherTarget> getDefaultLauncherTarget, RumbleManager rumbleManager) {
   super(intake, lift, feeder, launcher, getDefaultLauncherTarget, rumbleManager);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_feeder.hasNote();
  }
}
