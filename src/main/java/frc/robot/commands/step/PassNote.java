// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;

// TODO: Implement actual control logic
public class PassNote extends Command {
  private Intake m_intake;
  private Lift m_lift;
  private Launcher m_launcher;
  private Feeder m_feeder;

  /** Creates a new PassNote. */
  public PassNote(Intake intake, Lift lift, Feeder feeder, Launcher launcher) {
    m_intake = intake;
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(intake, lift, feeder, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.moveToHeight(LiftConstants.PassNoteHeight);
    m_launcher.setTiltAngle(LauncherConstants.PassNoteAngle);

    if (m_lift.atTargetHeight(LiftConstants.PassNoteHeight) && m_launcher.atTargetAngle(LauncherConstants.PassNoteAngle)) {
      m_feeder.setFeederPower(-1.0);
    } else {
      m_feeder.setFeederPower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePower(0);
    m_launcher.setLauncherPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
