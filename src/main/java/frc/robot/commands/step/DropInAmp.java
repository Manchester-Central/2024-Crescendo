// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;

// TODO: Implement actual control logic
public class DropInAmp extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private Feeder m_feeder;
  private Timer m_spitTimer = new Timer();
  private boolean m_hasLostNote = false;

  /** Creates a new DropInAmp. */
  public DropInAmp(Lift lift, Launcher launcher, Feeder feeder) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(lift, launcher, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spitTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_spitTimer.hasElapsed(1)) {
      m_lift.moveToHeight(LiftConstants.IntakeHeightMeters);
      return;
    }
    m_lift.moveToHeight(LiftConstants.AmpMeters);
    m_launcher.setLauncherPower(0.0);
    m_launcher.setTiltAngle(LauncherConstants.AmpAngle);
    if (m_lift.atTargetHeight(LiftConstants.AmpMeters) && m_launcher.atTargetAngle(LauncherConstants.AmpAngle)) {
      m_feeder.setFeederPower(-1.0);
    }
    if (!m_hasLostNote && !m_feeder.hasNoteAtSecondary() && !m_feeder.hasNoteAtPrimary()) {
      m_hasLostNote = true;
      m_spitTimer.start();
    }
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
