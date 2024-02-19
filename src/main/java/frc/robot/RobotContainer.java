// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.DefaultFeederCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultLauncherCommand;
import frc.robot.commands.DefaultLiftCommand;
import frc.robot.commands.DefaultVisionCommand;
import frc.robot.commands.DriveToLocation;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DropInAmp;
import frc.robot.commands.FireIntoAmp;
import frc.robot.commands.Launch;
import frc.robot.commands.LiftClimbAndPull;
import frc.robot.commands.Outtake;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.auto.AimForNote;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Mode;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpeakerFocus;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;
import frc.robot.util.FieldPose2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(0,0.2,0.2);
  private Gamepad m_operator = new Gamepad(1);
  public static Gamepad SimKeyboard = new Gamepad(2);
  private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private SwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Vision m_vision = new Vision("limelight");
  private Intake m_intake = new Intake();
  private Lift m_lift = new Lift();
  private Feeder m_feeder = new Feeder();
  private Launcher m_launcher = new Launcher();

  public RobotContainer() {
     m_swerveDrive.resetPose(FieldPose2024.TestStart.getCurrentAlliancePose());
     configureBindings();
     m_autoBuilder.registerCommand("drive", (pc) -> DriveToLocation.createAutoCommand(pc, m_swerveDrive) );
     m_autoBuilder.registerCommand("resetPosition", (pc) -> ResetPosition.createAutoCommand(pc, m_swerveDrive));
     m_autoBuilder.registerCommand("launch", (pc) -> Launch.createAutoCommand(pc, m_lift, m_launcher, m_feeder));
     m_autoBuilder.registerCommand("intake", (pc) -> RunIntake.createAutoCommand(pc, m_intake, m_lift, m_launcher, m_feeder));
     m_autoBuilder.registerCommand("driveAndIntake", (pc)-> AutoUtil.driveAndIntake(pc, m_swerveDrive, m_intake, m_lift, m_launcher, m_feeder));
  }
  

  private void configureBindings() {
    var robotRelativeDrive = new RobotRelativeDrive(m_driver, m_swerveDrive);
    var driverRelativeDrive = new DriverRelativeDrive(m_driver, m_swerveDrive);
    var slowCommand = new StartEndCommand(
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 0.5; 
         BaseSwerveDrive.RotationSpeedModifier = 0.5;
      },      
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 1.0; 
         BaseSwerveDrive.RotationSpeedModifier = 1.0;
      }
    );
    var frozoneSlowCommand = new StartEndCommand(
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 0.25; 
         BaseSwerveDrive.RotationSpeedModifier = 0.25;
      },      
      () -> {
         BaseSwerveDrive.TranslationSpeedModifier = 1.0; 
         BaseSwerveDrive.RotationSpeedModifier = 1.0;
      }
    );

    // Default commands
    m_vision.setDefaultCommand(new DefaultVisionCommand(m_vision, m_swerveDrive));
    m_swerveDrive.setDefaultCommand(driverRelativeDrive);
    // m_swerveDrive.setDefaultCommand(robotRelativeDrive);
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake));
    m_lift.setDefaultCommand(new DefaultLiftCommand(m_lift, m_operator));
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher, m_operator));
    m_feeder.setDefaultCommand(new DefaultFeederCommand(m_feeder));

    // Driver
    m_driver.back().onTrue(robotRelativeDrive);
    m_driver.start().onTrue(driverRelativeDrive);

    m_driver.povUp().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180))));
    
    m_driver.b().whileTrue(new SpeakerFocus(m_swerveDrive, m_driver));
    m_driver.y().whileTrue(new LiftClimbAndPull(m_lift, m_swerveDrive));
    m_driver.x().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_swerveDrive));
    //m_driver.a().whileTrue(new RunIntake(m_intake, m_lift, m_launcher, m_feeder));
    //m_driver.b().whileTrue(new Outtake(m_intake, m_lift, m_launcher, m_feeder));

    m_driver.leftBumper().whileTrue(new SpeakerFocus(m_swerveDrive, m_driver));
    m_driver.leftTrigger().whileTrue(slowCommand);
    m_driver.rightBumper().whileTrue(frozoneSlowCommand);
    //m_driver.rightTrigger().whileTrue(new Launch(m_lift, m_launcher, m_feeder));

    m_driver.leftStick().whileTrue(slowCommand);
    m_driver.rightStick().whileTrue(frozoneSlowCommand);

    // Operator
    // m_operator.a().whileTrue(new RunIntake(m_intake, m_lift, m_launcher, m_feeder));
    // m_operator.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder));
    // m_operator.rightTrigger().whileTrue(new Launch(m_lift, m_launcher, m_feeder));

    m_operator.leftTrigger().whileTrue(new RunCommand(()-> {
      m_intake.setIntakePower(0.7);
      m_feeder.setFeederPower(0.7);
    }, m_intake, m_feeder));

     m_operator.rightTrigger().whileTrue(new RunCommand(()-> {
      m_launcher.setLauncherPower(0.7);
      m_feeder.setFeederPower(1.0);
    }, m_launcher, m_feeder));

    m_operator.leftBumper().whileTrue(new RunCommand(()-> {
      m_intake.setIntakePower(-0.2);
      m_feeder.setFeederPower(-0.2);
      m_launcher.setLauncherPower(-0.2);
    }, m_intake, m_feeder, m_launcher));
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
