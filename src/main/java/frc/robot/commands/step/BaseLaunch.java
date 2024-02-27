// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.Optional;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TableData;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

// TODO: Implement actual control logic
public abstract class BaseLaunch extends Command {
  protected Lift m_lift;
  protected Launcher m_launcher;
  protected Feeder m_feeder;

  /** Creates a new Lanch Partay. */
  public BaseLaunch(Lift lift, Launcher launcher, Feeder feeder) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(lift, launcher, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  protected void noTargetBehavior() {
    m_lift.setSpeed(0);
    m_launcher.setTiltSpeed(0);
    m_launcher.setLauncherPower(0);
    m_feeder.setFeederPower(0);

  }

  protected abstract Optional<TableData> getTargets();

  protected abstract boolean isClearToLaunch();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
      m_feeder.setFeederPower(1.0);
    }
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
