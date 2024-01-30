// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(0);
  private Gamepad m_operator = new Gamepad(1);

  private BaseSwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));
    m_driver.a().onTrue(new InstantCommand(() -> m_swerveDrive.recalibrateModules()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
