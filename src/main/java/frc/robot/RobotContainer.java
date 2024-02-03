// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.DriveToLocation;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(0);
  private Gamepad m_operator = new Gamepad(1);
  private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private BaseSwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Intake m_Intake = new Intake();

  public RobotContainer() {
    configureBindings();

    m_autoBuilder.registerCommand("drive", (pc) -> DriveToLocation.createAutoCommand(pc, m_swerveDrive) );
    m_autoBuilder.registerCommand("resetPosition", (pc) -> ResetPosition.createAutoCommand(pc, m_swerveDrive));
  }
  

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));
    m_Intake.setDefaultCommand(new RunCommand(()-> m_Intake.runSpeed(0.0), m_Intake));
    m_driver.a().onTrue(new InstantCommand(() -> m_swerveDrive.recalibrateModules()));
    m_driver.povUp().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(0))));
    m_driver.b().whileTrue(new InstantCommand(() -> m_swerveDrive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))).andThen( new DriveToLocation(new Pose2d(1, 0, Rotation2d.fromDegrees(0)), m_swerveDrive)));
    m_driver.leftTrigger().whileTrue(new StartEndCommand(
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 0.5; 
         BaseSwerveDrive.RotationSpeedModifier = 0.5;
      },      
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 1.0; 
         BaseSwerveDrive.RotationSpeedModifier = 1.0;
      } 
    ));
    m_driver.rightBumper().whileTrue(new RunCommand(()-> m_Intake.runSpeed(0.3), m_Intake));
  }

  public Command getAutonomousCommand() {
    return m_autoBuilder.createAutoCommand();
  }
}
