// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;

import com.chaos131.util.DashboardNumber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherTarget;

// TODO: Implement actual control logic
public abstract class BaseLaunch extends Command {
  protected Lift m_lift;
  protected Launcher m_launcher;
  protected Feeder m_feeder;
  protected Intake m_intake;

  private Timer m_launchTimer = new Timer();
  private boolean m_hasLostNote = false;

  private DashboardNumber m_feederLaunchSpeed = new DashboardNumber("Feeder Launch Speed", 1.0, true, (Double newSpeed)->{});
  private DashboardNumber m_trapLaunchSpeed = new DashboardNumber("Trap Launch Speed", 1.0, true, (Double newSpeed)->{});

  /** Creates a new Lanch Partay. */
  public BaseLaunch(Lift lift, Launcher launcher, Feeder feeder, Intake intake) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    m_intake = intake;
    addRequirements(lift, launcher, feeder, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launchTimer.stop();
    m_launchTimer.reset();
    m_hasLostNote = false;
  }

  protected void noTargetBehavior() {
    m_lift.setSpeed(0);
    m_launcher.setTiltSpeed(0);
   // m_launcher.setLauncherPower(0);
    m_launcher.setLauncherRPM(3000, 3000);
    m_feeder.setFeederPower(0);

  }

  protected abstract Optional<LauncherTarget> getTargets();

  protected abstract boolean isClearToLaunch();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakePower(1.0);
    if (!m_hasLostNote && !m_feeder.hasNote() && !m_intake.hasNote()) {
      m_hasLostNote = true;
      m_launchTimer.start();
    } else if (m_feeder.hasNote() || m_intake.hasNote()) {
      m_hasLostNote = false;
      m_launchTimer.stop();
      m_launchTimer.reset();
    }
    var targetOptional = getTargets();
    if (targetOptional.isEmpty()) {
      noTargetBehavior();
      return;
    }
    var targets = targetOptional.get();
    SmartDashboard.putString("Flywheel data", targets.toString());
    var targetHeight = targets.getHeightMeters();
    var targetSpeed = targets.getLauncherSpeedRPM();
    var speedOffset = targets.getSpeedOffsetRPM();
    var targetSpeedLeft = targetSpeed + speedOffset;
    var targetSpeedRight = targetSpeed - speedOffset;
    var targetTilt = targets.getTiltAngle();
    m_lift.moveToHeight(targetHeight);
    m_launcher.setLauncherRPM(targetSpeedLeft, targetSpeedRight);
    m_launcher.setTiltAngle(targetTilt);
    if (isClearToLaunch() && m_lift.atTargetHeight(targetHeight) && m_launcher.atTargetAngle(targetTilt) && m_launcher.atTargetRPM(targetSpeedLeft, targetSpeedRight)) {
      m_feeder.setFeederPower(m_feederLaunchSpeed.get(),m_trapLaunchSpeed.get());
    } else {
      m_feeder.grabAndHoldPiece(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setLauncherPower(0);
    m_feeder.setFeederPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_launchTimer.hasElapsed(0.05);
  }
}
