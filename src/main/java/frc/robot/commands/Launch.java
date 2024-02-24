// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

// TODO: Implement actual control logic
public class Launch extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private Feeder m_feeder;
  private FlywheelTable m_flywheelTable;
  private SwerveDrive m_swerveDrive;

  /** Creates a new Lanch Partay. */
  public Launch(Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTable, SwerveDrive swerveDrive) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    m_flywheelTable = flywheelTable;
    m_swerveDrive = swerveDrive;
    addRequirements(lift, launcher, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetOptional = m_flywheelTable.getIdealTarget(FieldPose2024.Speaker.distanceTo(m_swerveDrive.getPose()));
    if (targetOptional.isEmpty()) {
      m_lift.setSpeed(0);
      m_launcher.setTiltSpeed(0);
      m_launcher.setLauncherPower(0);
      m_feeder.setFeederPower(0);
      return;
    }
    var targets = targetOptional.get();
    SmartDashboard.putString("Flywheel data", targets.toString());
    var targetHeight = targets.getHeightMeters();
    var targetSpeed = targets.getLauncherSpeedRPM();
    var targetTilt = targets.getTiltAngle();
    m_lift.moveToHeight(targetHeight);
    m_launcher.setLauncherRPM(targetSpeed);
    m_launcher.setTiltAngle(targetTilt);
    if (m_lift.atTargetHeight(targetHeight) && m_launcher.atTargetAngle(targetTilt) && m_launcher.atTargetRPM(targetSpeed)) {
      m_feeder.setFeederPower(1.0);
    }
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTable, SwerveDrive swerveDrive){
    return new Launch(lift, launcher, feeder, flywheelTable, swerveDrive);
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
