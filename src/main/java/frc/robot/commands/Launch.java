// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class Launch extends BaseLaunch {
  private FlywheelTable m_flywheelTable;
  private Vision m_vision;

  /** Creates a new Lanch Partay. */
  public Launch(Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTable, Vision vision) {
    super (lift, launcher, feeder);
    m_flywheelTable = flywheelTable;
    m_vision = vision;
    addRequirements(lift, launcher, feeder);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTable, Vision vision){
    return new Launch(lift, launcher, feeder, flywheelTable, vision);
  }

  @Override
  protected Optional<TableData> getTargets() {
    var pose = m_vision.getPose();
    if (pose == null) {
      return Optional.empty(); 
    }
    return m_flywheelTable.getIdealTarget(FieldPose2024.Speaker.distanceTo(pose));
  }
}
