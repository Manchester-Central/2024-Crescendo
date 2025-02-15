// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chaos131.logging.LogManager;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LogManager m_logManager = LogManager.getInstance();

  private Timer m_disabledTimer = new Timer();

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    var pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    // TODO: Replace this with the code in CHAOS-Shared-Code
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
        // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        // setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_robotContainer = new RobotContainer(pdh);
    m_logManager.writeHeaders();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
    m_logManager.update();
  }

  @Override
  public void disabledInit() {
    m_disabledTimer.reset();
    m_disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (m_disabledTimer.hasElapsed(10)) {
      m_robotContainer.delayedDisableInit();
      m_disabledTimer.stop();
      m_disabledTimer.reset();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.autoAndTeleopInit(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.autoAndTeleopInit(false);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.demoInit();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
