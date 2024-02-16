// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;

// TODO: Implement actual control logic
public class Launch extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private Feeder m_feeder;

  /** Creates a new Lanch Partay. */
  public Launch(Lift lift, Launcher launcher, Feeder feeder) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(lift, launcher, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.moveToHeight(LiftConstants.DefaultLaunchMeters);
    m_launcher.setLauncherRPM(LauncherConstants.MaxRPM); // TODO: value should be determined from limelight/odometry
    m_launcher.setLauncherAngle(LauncherConstants.BumperShotAngle); // TODO: value should be determined from limelight/odometry
    if (m_lift.atTargetHeight(LiftConstants.DefaultLaunchMeters) && m_launcher.atTargetAngle(LauncherConstants.BumperShotAngle) && m_launcher.atTargetRPM(LauncherConstants.MaxRPM)) {
      m_feeder.setFeederPower(0.3);
    }
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Lift lift, Launcher launcher, Feeder feeder){
    return new Launch(lift, launcher, feeder);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setLauncherPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
