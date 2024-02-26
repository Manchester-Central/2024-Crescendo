// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TableData;
import frc.robot.subsystems.Vision;
import frc.robot.util.FieldPose2024;

/**
 * Creates a Flywheel Table Launch command
 */
public class Launch extends BaseLaunch {
  private FlywheelTable m_flywheelTableLowerHeight;
  private FlywheelTable m_flywheelTableUpperHeight;
  private Vision m_vision;
  private boolean m_beenAboveThreshold = false;

  /** Creates a new Lanch Partay. */
  public Launch(Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTableLowerHeight, FlywheelTable flywheelTableUpperHeight, Vision vision) {
    super (lift, launcher, feeder);
    m_flywheelTableLowerHeight = flywheelTableLowerHeight;
    m_flywheelTableUpperHeight = flywheelTableUpperHeight;
    m_vision = vision;
    addRequirements(lift, launcher, feeder);
  }

  @Override
  public void initialize() {
    m_beenAboveThreshold = false;
    super.initialize();
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Lift lift, Launcher launcher, Feeder feeder, FlywheelTable flywheelTableLowerHeight, FlywheelTable flywheelTableUpperHeight, Vision vision){
    return new Launch(lift, launcher, feeder, flywheelTableLowerHeight, flywheelTableUpperHeight, vision);
  }

  @Override
  protected Optional<TableData> getTargets() {
    var pose = m_vision.getPose();
    if (pose == null) {
      return Optional.empty(); 
    }
    var distanceMeters = FieldPose2024.Speaker.distanceTo(pose);
    if (distanceMeters >= CommandConstants.LaunchUpperFlywheelTableThresholdMeters) {
      m_beenAboveThreshold = true;
    }
    return (m_beenAboveThreshold ? m_flywheelTableUpperHeight : m_flywheelTableLowerHeight).getIdealTarget(distanceMeters);
  }
}
