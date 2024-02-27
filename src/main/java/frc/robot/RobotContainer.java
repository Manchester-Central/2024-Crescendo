// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

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
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DashboardLaunch;
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
import frc.robot.commands.SimpleControl;
import frc.robot.commands.SpeakerFocus;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;
import frc.robot.util.FieldPose2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(0,10,10);
  private Gamepad m_operator = new Gamepad(1);
  public static Gamepad SimKeyboard = new Gamepad(2);
  private Gamepad m_tester = new Gamepad(3);
  private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private SwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Vision m_vision = new Vision("limelight-front");
  private Intake m_intake = new Intake();
  private Lift m_lift = new Lift();
  private Feeder m_feeder = new Feeder();
  private Launcher m_launcher = new Launcher();
  private FlywheelTable m_flywheelTableLowerHeight = new FlywheelTable(FlywheelTable.FlywheelTableLowerHeight);
  private FlywheelTable m_flywheelTableUpperHeight = new FlywheelTable(FlywheelTable.FlywheelTableUpperHeight);

  public RobotContainer() {
    m_swerveDrive.resetPose(FieldPose2024.TestStart.getCurrentAlliancePose());
    configureBindings();
    m_autoBuilder.registerCommand("drive", (pc) -> DriveToLocation.createAutoCommand(pc, m_swerveDrive) );
    m_autoBuilder.registerCommand("resetPosition", (pc) -> ResetPosition.createAutoCommand(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("launch", (pc) -> Launch.createAutoCommand(pc, m_lift, m_launcher, m_feeder, m_flywheelTableLowerHeight, m_flywheelTableUpperHeight, m_vision));
    m_autoBuilder.registerCommand("intake", (pc) -> RunIntake.createAutoCommand(pc, m_intake, m_lift, m_feeder));
    m_autoBuilder.registerCommand("driveAndIntake", (pc)-> AutoUtil.driveAndIntake(pc, m_swerveDrive, m_intake, m_lift, m_feeder));
    m_autoBuilder.registerCommand("driveAndIntakeSimple", (pc)-> AutoUtil.driveAndIntakeSimple(pc, m_swerveDrive, m_intake, m_lift, m_launcher, m_feeder));
    m_autoBuilder.registerCommand("flyWheelOn", (pc) -> new SimpleControl().flywheel(m_launcher, 1.0));
    m_autoBuilder.registerCommand("flyWheelAndFeederOn", (pc) -> new SimpleControl().flywheel(m_launcher, 1.0).feeder(m_feeder, 1.0));
    m_autoBuilder.registerCommand("tiltDown", (pc) -> new StartEndCommand(() -> m_launcher.setTiltSpeed(-0.08), () -> m_launcher.setTiltSpeed(0), m_launcher));
    m_autoBuilder.registerCommand("simpleControl", (pc) -> SimpleControl.createAutoCommand(pc, m_intake, m_feeder, m_launcher, m_lift));
    if(Robot.isSimulation()) {
      // Stores a lambda to collect the pose without needing an explicit swerve drive reference
      m_vision.prepSimulation(() -> m_swerveDrive.getPose()); 
    }
  }
  


  private void configureBindings() {
    var robotRelativeDrive = new RobotRelativeDrive(m_driver, m_swerveDrive);
    var driverRelativeDrive = new DriverRelativeDrive(m_driver, m_swerveDrive);

    m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier);

    var slowCommand = new StartEndCommand(
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.SlowSpeedModifier),
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier)
    );
    var frozoneSlowCommand = new StartEndCommand(
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.SuperSlowSpeedModifier),
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier)
    );

    // Default commands
    m_vision.setDefaultCommand(new DefaultVisionCommand(m_vision, m_swerveDrive));
    m_swerveDrive.setDefaultCommand(driverRelativeDrive);
    // m_swerveDrive.setDefaultCommand(robotRelativeDrive);
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake));
    m_lift.setDefaultCommand(new DefaultLiftCommand(m_lift, m_operator));
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher, m_operator));
    m_feeder.setDefaultCommand(new DefaultFeederCommand(m_feeder, m_tester));

    // Tester
    m_tester.a().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(15)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    m_tester.b().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(40)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    m_tester.x().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.2), () -> m_lift.setSpeed(0), m_lift));
    m_tester.y().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.6), () -> m_lift.setSpeed(0), m_lift));
   // m_tester.rightBumper().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(2000), () -> m_launcher.setLauncherPower(0), m_launcher));
   // m_tester.rightTrigger().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(5000), () -> m_launcher.setLauncherPower(0), m_launcher));

    // Driver
    m_driver.back().onTrue(robotRelativeDrive);
    m_driver.start().onTrue(driverRelativeDrive);

    // var updateHeading = (double blueAngle) -> {
    //   return new InstantCommand(() -> m_swerveDrive.resetHeading())
    // }

    m_driver.povUp().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180))));
    // m_driver.povDown().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180).)));
    // m_driver.povLeft().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180))));
    // m_driver.povRight().onTrue(new InstantCommand(() -> m_swerveDrive.resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180))));


    m_driver.a().whileTrue(new StartEndCommand(() -> Lift.SafeftyLimtEnabled = false, () -> Lift.SafeftyLimtEnabled = true)); // The driver can allow the operator to extend the lift past the safety zone
    m_driver.b().whileTrue(new SpeakerFocus(m_swerveDrive, m_driver, m_vision));
   // m_driver.y().whileTrue(new LiftClimbAndPull(m_lift, m_swerveDrive));
   // m_driver.x().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_swerveDrive));
    // TODO: do the other 3 directions (Left, Right, Down)

    //m_driver.x().whileTrue(new SpeakerFocus(m_swerveDrive, m_driver));
    //m_driver.a().whileTrue(new RunIntake(m_intake, m_lift, m_launcher, m_feeder));
    //m_driver.b().whileTrue(new Outtake(m_intake, m_lift, m_launcher, m_feeder));

    m_driver.leftBumper().whileTrue(new SpeakerFocus(m_swerveDrive, m_driver, m_vision));
    m_driver.leftTrigger().whileTrue(slowCommand);
    m_driver.rightBumper().whileTrue(frozoneSlowCommand);
    m_driver.rightTrigger().whileTrue(new Launch(m_lift, m_launcher, m_feeder, m_flywheelTableLowerHeight, m_flywheelTableUpperHeight, m_vision));

    m_driver.leftStick().whileTrue(slowCommand);
    m_driver.rightStick().whileTrue(frozoneSlowCommand);

    // Operator
    // m_operator.a().whileTrue(new RunIntake(m_intake, m_lift, m_launcher, m_feeder));
    // m_operator.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder));
    // m_operator.rightTrigger().whileTrue(new Launch(m_lift, m_launcher, m_feeder));

    m_operator.back().whileTrue(frozoneSlowCommand);

   // m_operator.a().whileTrue(new SimpleControl().intake(m_intake, 0.7)); // Simple Intake
   m_operator.a().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(LiftConstants.MaxHeightMeters), () -> m_lift.setSpeed(0), m_lift));
    m_operator.b().whileTrue(new SimpleControl().intake(m_intake, -0.2).feeder(m_feeder, -0.2).flywheel(m_launcher, -0.2)); // Simple spit
    m_operator.x().whileTrue(new RunIntake(m_intake, m_lift, m_feeder));
    m_operator.y().whileTrue(new DashboardLaunch(m_lift, m_launcher, m_feeder));

    m_operator.leftBumper().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(LiftConstants.MinLaunchOverHeightMeters), () -> m_lift.setSpeed(0), m_lift)); // Simple Amp
    m_operator.leftTrigger().whileTrue(new SimpleControl().feeder(m_feeder, -1.0, 1.0).flywheel(m_launcher, -1.0)); // Simple Amp

    m_operator.rightBumper().whileTrue(new SimpleControl().flywheel(m_launcher, 1.0)); // Simple Prepare Flywheel
    m_operator.rightTrigger().whileTrue(new SimpleControl().feeder(m_feeder, 1.0).flywheel(m_launcher, 1.0)); // Simple Speaker Launch

    m_operator.povLeft().whileTrue(new StartEndCommand(() -> DefaultLiftCommand.MaxLiftSpeed = 0.1, () -> DefaultLiftCommand.MaxLiftSpeed = 1)); // Slower manual lift speed
  }
  
  public Command getAutonomousCommand() {
    return m_autoBuilder.createAutoCommand();
  }

  public void periodic() {
    // Chaosboard expects: [intakePower, liftHeightMeters, launcherAngleDegrees, feederPower, launcherPower, atFeederPrimary, atFeederSecondary]
    double[] RobotState = {
      m_intake.getCurrentIntakePower(),
      m_lift.getCurrentHeightMeters(),
      m_launcher.getAbsoluteTiltAngle().getDegrees(),
      m_feeder.getCurrentFeederPower(),
      m_launcher.getCurrentLauncherPower(),
      m_feeder.hasNoteAtPrimary() ? 1 : 0, 
      m_feeder.hasNoteAtSecondary() ? 1 : 0
    };
    SmartDashboard.putNumberArray("Robot2024/State", RobotState);

    var pose = m_vision.getPose();
    var distanceToSpeaker = -1.0;
    if(pose != null){
      distanceToSpeaker = FieldPose2024.Speaker.distanceTo(pose);
    }
    SmartDashboard.putNumber("Distance to Speaker", distanceToSpeaker);
  }
}
