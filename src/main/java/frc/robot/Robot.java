// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.logging.LogManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LogManager m_logManager = LogManager.getInstance();

  private Timer m_disabledTimer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
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
