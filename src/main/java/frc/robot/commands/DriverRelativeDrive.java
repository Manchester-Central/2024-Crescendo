// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;

public class DriverRelativeDrive extends Command {
  private Gamepad m_driver;
  private BaseSwerveDrive m_swerveDrive;
  /** Creates a new DriverRelativeDrive. */
  public DriverRelativeDrive(Gamepad driver, BaseSwerveDrive swerveDrive) {
    m_driver = driver;
    m_swerveDrive = swerveDrive;

    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driverModeInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.moveFieldRelative(m_driver.getLeftY(), -m_driver.getLeftX(), -m_driver.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
