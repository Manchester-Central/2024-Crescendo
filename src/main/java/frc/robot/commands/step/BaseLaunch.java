// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.util.DashboardNumber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebugConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;

public abstract class BaseLaunch extends Command {
  protected final DecimalFormat m_formatter = new DecimalFormat("#.###");
  protected boolean kTuningEnabled = DebugConstants.LauncherModelDebugEnable;
  protected Lift m_lift;
  protected Launcher m_launcher;
  protected Feeder m_feeder;
  protected Intake m_intake;
  protected Optional<LauncherTarget> m_currentTarget = Optional.empty();
  protected Debouncer m_goodToLaunch = new Debouncer(0.05, DebounceType.kBoth); // Only launch if it's been safe to launch for 0.05 seconds

  private Timer m_launchTimer = new Timer();
  private boolean m_hasLostNote = false;

  private DashboardNumber m_feederLaunchSpeed = new DashboardNumber("Feeder Launch Speed", 1.0, kTuningEnabled, (Double newSpeed)->{});
  private DashboardNumber m_trapLaunchSpeed = new DashboardNumber("Trap Launch Speed", 1.0, kTuningEnabled, (Double newSpeed)->{});

  private Supplier<LauncherTarget> m_getDefaultLauncherTarget;

  /** Creates a new Lanch Partay. */
  public BaseLaunch(Lift lift, Launcher launcher, Feeder feeder, Intake intake, Supplier<LauncherTarget> getDefaultLauncherTarget) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    m_intake = intake;
    m_getDefaultLauncherTarget = getDefaultLauncherTarget;
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
    var launcherTarget = m_getDefaultLauncherTarget.get();
    m_launcher.setTiltAngle(launcherTarget.getTiltAngle());
    m_launcher.setLauncherRPM(launcherTarget.getLeftLauncherSpeedRPM(), launcherTarget.getRightLauncherSpeedRPM());
    m_feeder.setFeederPower(0);

  }

  protected abstract Optional<LauncherTarget> getTargets();

  protected abstract boolean isClearToLaunch();

  protected List<String> getLaunchErrors() {
    if (m_currentTarget.isEmpty()) {
      return new ArrayList<String>(List.of("No Launch Target"));
    }
    var target = m_currentTarget.get();
    var liftError = m_lift.getCurrentHeightMeters() - target.getHeightMeters();
    var leftLauncherError = m_launcher.getLeftLauncherRPM() - target.getLeftLauncherSpeedRPM();
    var rightLauncherError = m_launcher.getRightLauncherRPM() - target.getRightLauncherSpeedRPM();
    var tiltError = m_launcher.getEncoderTiltAngle().minus(target.getTiltAngle()).getDegrees();
    return new ArrayList<String>(List.of(
      formatError("Left Launcher", leftLauncherError),
      formatError("Right Launcher", rightLauncherError),
      formatError("Lift", liftError),
      formatError("Tilt", tiltError)
    ));
  }

  protected String formatError(String errorName, double value) {
    return errorName + " Error: " + m_formatter.format(value);
  }

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
    m_currentTarget = getTargets();
    LauncherModel.publishTargetToDashboard(m_currentTarget, "Base Launch Target");
    if(kTuningEnabled) {
      SmartDashboard.putStringArray("BaseLaunch/ErrorValues", getLaunchErrors().stream().toArray(String[]::new));
    }
    if (m_currentTarget.isEmpty()) {
      noTargetBehavior();
      m_goodToLaunch.calculate(false);
      return;
    }
    var targets = m_currentTarget.get();
    var targetHeight = targets.getHeightMeters();
    var targetSpeedLeft = targets.getLeftLauncherSpeedRPM();
    var targetSpeedRight = targets.getRightLauncherSpeedRPM();
    var targetTilt = targets.getTiltAngle();
    m_lift.moveToHeight(targetHeight);
    m_launcher.setLauncherRPM(targetSpeedLeft, targetSpeedRight);
    m_launcher.setTiltAngle(targetTilt);
    var isGoodToLaunch = m_goodToLaunch.calculate(isClearToLaunch() && m_lift.atTargetHeight(targetHeight) && m_launcher.atTargetAngle(targetTilt) && m_launcher.atTargetRPM(targetSpeedLeft, targetSpeedRight));
    if (isGoodToLaunch) {
      m_feeder.setFeederPower(getFeederLaunchSpeed(), getTrapLaunchSpeed());
    } else {
      m_feeder.grabAndHoldPiece(0.3);
    }
  }

  protected double getTrapLaunchSpeed(){
    return m_trapLaunchSpeed.get();
  }

  protected double getFeederLaunchSpeed(){
    return m_feederLaunchSpeed.get();
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
    // If teleop or demo mode, run the launch command until the user releases the button
    if (DriverStation.isTeleop() || DriverStation.isTest()) {
      return false;
    }
    return m_launchTimer.hasElapsed(0.05);
  }
}
