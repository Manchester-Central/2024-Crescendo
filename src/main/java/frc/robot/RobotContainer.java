// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Function;

// import com.chaos131.auto.AutoBuilder;
import com.chaos131.gamepads.Gamepad;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.commands.complex.FireIntoAmp;
import frc.robot.commands.defaults.DefaultFeederCommand;
import frc.robot.commands.defaults.DefaultIntakeCommand;
import frc.robot.commands.defaults.DefaultLauncherCommand;
import frc.robot.commands.defaults.DefaultLiftCommand;
import frc.robot.commands.defaults.DefaultVisionCommand;
import frc.robot.commands.simpledrive.DriveToLocation;
import frc.robot.commands.simpledrive.DriverRelativeDrive;
import frc.robot.commands.simpledrive.DriverRelativeSetAngleDrive;
import frc.robot.commands.simpledrive.ResetPosition;
import frc.robot.commands.simpledrive.RobotRelativeDrive;
import frc.robot.commands.simpledrive.UpdateHeading;
import frc.robot.commands.step.DashboardLaunch;
import frc.robot.commands.step.DropInAmp;
import frc.robot.commands.step.FocusAndLaunch;
import frc.robot.commands.step.PassNote;
//import frc.robot.commands.step.Launch;
import frc.robot.commands.step.RunIntake;
import frc.robot.commands.step.SeaCucumber;
import frc.robot.commands.step.SimpleControl;
//import frc.robot.commands.step.SpeakerFocus;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;
import frc.robot.util.DriveDirection;
import frc.robot.util.FieldPose2024;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(ControllerConstants.DriverPort, 10, 10);
  private Gamepad m_operator = new Gamepad(ControllerConstants.OperatorPort);
  public static Gamepad SimKeyboard;
  private Gamepad m_tester;
  // private final AutoBuilder m_autoBuilder = new AutoBuilder();

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

  private final SendableChooser<Command> m_pathPlannerChooser;

  private double m_noteSeenTime = 0;
  private boolean m_noteRumbleDebounce = false;

  private Command m_slowCommand = new StartEndCommand(
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.SlowSpeedModifier),
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier)
  );

  public RobotContainer() {
    m_swerveDrive.resetPose(FieldPose2024.TestStart.getCurrentAlliancePose());
    configureBindings();
    // m_autoBuilder.registerCommand("drive", (pc) -> DriveToLocation.createAutoCommand(pc, m_swerveDrive) );
    // m_autoBuilder.registerCommand("resetPosition", (pc) -> ResetPosition.createAutoCommand(pc, m_swerveDrive));
    // m_autoBuilder.registerCommand("launch", (pc) -> FocusAndLaunch.createAutoCommand(pc, m_lift, m_launcher, m_feeder, m_flywheelTableLowerHeight, m_flywheelTableUpperHeight, m_vision, m_swerveDrive, m_driver));
    // m_autoBuilder.registerCommand("intake", (pc) -> RunIntake.createAutoCommand(pc, m_intake, m_lift, m_feeder, m_launcher));
    // m_autoBuilder.registerCommand("driveAndIntake", (pc)-> AutoUtil.driveAndIntake(pc, m_swerveDrive, m_intake, m_lift, m_feeder, m_launcher));
    // m_autoBuilder.registerCommand("driveAndIntakeSimple", (pc)-> AutoUtil.driveAndIntakeSimple(pc, m_swerveDrive, m_intake, m_lift, m_launcher, m_feeder));
    // m_autoBuilder.registerCommand("flyWheelOn", (pc) -> new SimpleControl().flywheel(m_launcher, 1.0));
    // m_autoBuilder.registerCommand("flyWheelAndFeederOn", (pc) -> new SimpleControl().flywheel(m_launcher, 1.0).feeder(m_feeder, 1.0));
    // m_autoBuilder.registerCommand("tiltDown", (pc) -> new StartEndCommand(() -> m_launcher.setTiltSpeed(-0.08), () -> m_launcher.setTiltSpeed(0), m_launcher));
    // m_autoBuilder.registerCommand("simpleControl", (pc) -> SimpleControl.createAutoCommand(pc, m_intake, m_feeder, m_launcher, m_lift));

    m_vision.updateAprilTagMode(m_swerveDrive.getPose());

    NamedCommands.registerCommand("launch", new FocusAndLaunch(m_lift, m_launcher, m_feeder, m_flywheelTableLowerHeight, m_flywheelTableUpperHeight, m_vision, m_swerveDrive, m_driver));
    NamedCommands.registerCommand("intake", new RunIntake(m_intake, m_lift, m_feeder, m_launcher));
    NamedCommands.registerCommand("intakeWait", new RunIntake(m_intake, m_lift, m_feeder, m_launcher).withTimeout(0.25));
    NamedCommands.registerCommand("seaCucumber", new SeaCucumber(m_intake, m_lift, m_feeder, m_launcher));
    // Build an auto chooser. This will use Commands.none() as the default option.
    m_pathPlannerChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
    SmartDashboard.putData("Auto Chooser", m_pathPlannerChooser);
  }

  private void configureBindings() {
    
    if (Robot.isSimulation()) {
      m_vision.prepSimulation(() -> m_swerveDrive.getPose()); 
      SimKeyboard = new Gamepad(ControllerConstants.SimKeyboardPort);
    }

    configureDefaultCommands();
    configureDriverCommands();
    configureOperatorCommands();

    if (Constants.DebugMode) {
      m_tester = new Gamepad(ControllerConstants.TesterPort);
      configureTesterCommands();
    }
  }

  private void configureDefaultCommands() {
    m_vision.setDefaultCommand(new DefaultVisionCommand(m_vision, m_swerveDrive));
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));
    // m_swerveDrive.setDefaultCommand(robotRelativeDrive);
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake));
    m_lift.setDefaultCommand(new DefaultLiftCommand(m_lift, m_operator));
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher, m_operator));
    m_feeder.setDefaultCommand(new DefaultFeederCommand(m_feeder, m_tester));
  }

  private void configureDriverCommands() {
    m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier);

    m_driver.back().onTrue(new RobotRelativeDrive(m_driver, m_swerveDrive)); // Robot Relative
    m_driver.start().onTrue(new DriverRelativeDrive(m_driver, m_swerveDrive)); // Drive Relative

    m_driver.povUp().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Away)); // 0 degrees for blue
    m_driver.povDown().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Towards)); // 180 degrees for blue
    m_driver.povLeft().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Left)); // 90 degrees for blue
    m_driver.povRight().onTrue(new UpdateHeading(m_swerveDrive, DriveDirection.Right)); // -90 degrees for blue

    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, Rotation2d.fromDegrees(-90), 1.0)); // Align angle to amp (but allow translation)
    m_driver.b().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageRight, 1.0)); // Align angle to stage left (but allow translation)
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageLeft, 1.0)); // Align angle to stage right (but allow translation)
    m_driver.y().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, () -> FieldPose2024.Source.getCurrentAlliancePose().getRotation(), 1.0));  // Align angle to HP (but allow translation)

    // m_driver.leftBumper().whileTrue(m_slowCommand); // Slow command (and max height shot?)
    m_driver.leftBumper().whileTrue(new PassNote(m_intake, m_lift, m_feeder, m_launcher));
    m_driver.leftTrigger().whileTrue(new RunIntake(m_intake, m_lift, m_feeder, m_launcher)); // Intake
    m_driver.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder)); // Amp score
    m_driver.rightTrigger() // Aim and launch at speaker 
      .whileTrue( 
        new FocusAndLaunch(m_lift, m_launcher, m_feeder, m_flywheelTableLowerHeight, m_flywheelTableUpperHeight, m_vision, m_swerveDrive, m_driver));

    m_driver.leftStick(); //
    m_driver.rightStick(); //
  }

  private void configureOperatorCommands() {
    m_operator.back(); // Enable Automation (?)
    m_operator.start(); // Disable Automation (?)

    m_operator.povUp().whileTrue(new SimpleControl().intake(m_intake, -0.2).feeder(m_feeder, -0.2).flywheel(m_launcher, -0.2)); // Reverse Intake (dumb)
    m_operator.povDown().whileTrue(new SimpleControl().intake(m_intake, 0.7)); // Intake (dumb)
    m_operator.povLeft().whileTrue(new SimpleControl().feeder(m_feeder, 0.3, 0)); // Position note for trap 
    m_operator.povRight().whileTrue(new SeaCucumber(m_intake, m_lift, m_feeder, m_launcher)); // 

    Function<Double, StartEndCommand> createGetHeightCommand = (Double height) -> new StartEndCommand(() -> m_lift.moveToHeight(height), () -> m_lift.setSpeed(0), m_lift);
    Function<Rotation2d, StartEndCommand> createGetTiltCommand = (Rotation2d angle) -> new StartEndCommand(() -> m_launcher.setTiltAngle(angle), () -> m_launcher.setTiltSpeed(0), m_launcher);
    m_operator.a().whileTrue(createGetHeightCommand.apply(LiftConstants.MinHeightMeters)); // Min height
    m_operator.b().whileTrue(createGetHeightCommand.apply(LiftConstants.AmpMeters)); // Amp Height
    m_operator.x().whileTrue(createGetHeightCommand.apply(LiftConstants.MinHeightMeters)); // HP Height
    m_operator.y().whileTrue(createGetHeightCommand.apply(LiftConstants.MaxHeightMeters).alongWith(createGetTiltCommand.apply(LauncherConstants.TrapAngle)).alongWith(m_slowCommand)); // Max height

    m_operator.leftBumper().whileTrue(new SimpleControl().feeder(m_feeder, -0.3, 0.3).flywheel(m_launcher, -0.3)); // Up Trap
    m_operator.leftTrigger().whileTrue( // Launch (dumb)
      (new SimpleControl().flywheel(m_launcher, 1.0).withTimeout(0.5))
      .andThen(new SimpleControl().feeder(m_feeder, 1.0).flywheel(m_launcher, 1.0)));
    m_operator.rightBumper().whileTrue(new SimpleControl().feeder(m_feeder, -0.1)); // Amp & Down Trap
    m_operator.rightTrigger().whileTrue(new RunIntake(m_intake, m_lift, m_feeder, m_launcher)); // Intake (smart)

    m_operator.leftStick(); //
    m_operator.rightStick(); //

    // left stick y - manual angle tilt
    // right stick y - manual lift height
  }

  private void configureTesterCommands() {
    m_tester.a().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(15)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    m_tester.b().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(40)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    m_tester.x().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.2), () -> m_lift.setSpeed(0), m_lift));
    m_tester.y().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.6), () -> m_lift.setSpeed(0), m_lift));
    // m_tester.leftBumper().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(2000), () -> m_launcher.setLauncherPower(0), m_launcher));
    // m_tester.leftTrigger().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(5000), () -> m_launcher.setLauncherPower(0), m_launcher));
    
    m_tester.rightTrigger().whileTrue(new DashboardLaunch(m_lift, m_launcher, m_feeder));
  }
  
  public Command getAutonomousCommand() {
    // return m_autoBuilder.createAutoCommand();
    return m_pathPlannerChooser.getSelected();
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

    var visionPose = m_vision.getPose();
    var distanceToSpeaker = -1.0;
    if(visionPose != null){
      if (!DriverStation.isTeleopEnabled()) {
        // temp change - disable odometry estimations when in teleop until we have tested better and got the back limelight tested too
        m_swerveDrive.addVisionMeasurement(m_vision.getPose(), m_vision.getLatencySeconds());
      }
      distanceToSpeaker = FieldPose2024.Speaker.distanceTo(visionPose);
    }else{
      visionPose = new Pose2d();
    }
    SmartDashboard.putNumber("Distance to Speaker", distanceToSpeaker);

    var drivePose = m_swerveDrive.getPose();
    double[] robotAndVision = {
      drivePose.getX(),
      drivePose.getY(),
      drivePose.getRotation().getDegrees(),
      visionPose.getX(),
      visionPose.getY(),
      visionPose.getRotation().getDegrees()
    };
    SmartDashboard.putNumberArray("Robot and Vision", robotAndVision);


    // Doing these rumbles in this periodic function so they trigger for regardless of what driver or operator command is being run
    handleDriverRumble();
    handleOperatorRumble();
  }

  // Rumble the driver controller inversely proportional to the battery voltage (up to a certain point) (so rumber more the lower the reported voltage gets)
  private void handleDriverRumble() {
    // var voltage = RobotController.getBatteryVoltage();
    // var voltageClamp = MathUtil.clamp(voltage, 8, 10);
    // var rumbleValue =  ((voltageClamp/-2) + 5);
    // m_driver.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
    if (m_feeder.hasNoteAtSecondary() && DriverStation.isTeleop()) {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
     
}

  // Rumble the operator controller for 0.25 seconds after getting a note and then stop until the next time
  private void handleOperatorRumble() {

    // If this is the first time seeing this note in the intake
    if(m_feeder.hasNote() && m_noteRumbleDebounce == false) {
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      m_noteSeenTime = Timer.getFPGATimestamp();
      m_noteRumbleDebounce = true;
    }

    // If we have no note or it's been more than 250` milliseconds since we first saw this note
    if(Timer.getFPGATimestamp() - m_noteSeenTime >= 0.25 && m_noteRumbleDebounce == true ) {
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    if(!m_feeder.hasNote()) {
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 0);
      m_noteRumbleDebounce = false;
    }
  }

  public void autoAndTeleopInit(boolean isAuto) {
    m_lift.changeNeutralMode(NeutralModeValue.Brake);
    m_vision.updateAprilTagMode(m_swerveDrive.getPose());
  
  }

  public void delayedDisableInit() {
    m_lift.changeNeutralMode(NeutralModeValue.Coast);
  }
}
