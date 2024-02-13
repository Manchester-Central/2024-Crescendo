// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.DefaultFeederCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultLauncherCommand;
import frc.robot.commands.DefaultLiftCommand;
import frc.robot.commands.DriveToLocation;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DropInAmp;
import frc.robot.commands.Launch;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(0);
  private Gamepad m_operator = new Gamepad(1);
  public static Gamepad SimKeyboard = new Gamepad(2);
  private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private BaseSwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Intake m_intake = new Intake();
  private Lift m_lift = new Lift();
  private Feeder m_feeder = new Feeder();
  private Launcher m_launcher = new Launcher();

  public RobotContainer() {
    configureBindings();

    m_autoBuilder.registerCommand("drive", (pc) -> DriveToLocation.createAutoCommand(pc, m_swerveDrive) );
    m_autoBuilder.registerCommand("resetPosition", (pc) -> ResetPosition.createAutoCommand(pc, m_swerveDrive));
  }
  

  private void configureBindings() {
    // Default commands
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));
    // m_swerveDrive.setDefaultCommand(new RobotRelativeDrive(m_driver, m_swerveDrive));
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake));
    m_lift.setDefaultCommand(new DefaultLiftCommand(m_lift));
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher));
    m_feeder.setDefaultCommand(new DefaultFeederCommand(m_feeder));

    // Driver
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

    // Operator
    m_operator.a().whileTrue(new RunIntake(m_intake, m_lift, m_launcher, m_feeder));
    m_operator.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder));
    m_operator.rightTrigger().whileTrue(new Launch(m_lift, m_launcher, m_feeder));
  }

  public Command getAutonomousCommand() {
    return m_autoBuilder.createAutoCommand();
  }

  public void periodic() {
    // Chaosboard expects: [intakePower, liftHeightMeters, launcherAngleDegrees, feederPower, launcherPower, atFeederPrimary, atFeederSecondary]
    double[] RobotState = {
      m_intake.getCurrentIntakePower(),
      m_lift.getCurrentHeightMeters(),
      m_launcher.getCurrentAngle().getDegrees(),
      m_feeder.getCurrentFeederPower(),
      m_launcher.getCurrentLauncherPower(),
      m_feeder.hasNoteAtPrimary() ? 1 : 0, 
      m_feeder.hasNoteAtSecondary() ? 1 : 0
    };
    SmartDashboard.putNumberArray("Robot2024/State", RobotState);
  }
}
