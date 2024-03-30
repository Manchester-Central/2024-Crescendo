// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

// import com.chaos131.auto.AutoBuilder;
import com.chaos131.gamepads.Gamepad;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.complex.FireIntoAmp;
import frc.robot.commands.defaults.DefaultFeederCommand;
import frc.robot.commands.defaults.DefaultIntakeCommand;
import frc.robot.commands.defaults.DefaultLauncherCommand;
import frc.robot.commands.defaults.DefaultLiftCommand;
import frc.robot.commands.defaults.DefaultVisionCommand;
import frc.robot.commands.simpledrive.DriverRelativeDrive;
import frc.robot.commands.simpledrive.DriverRelativeSetAngleDrive;
import frc.robot.commands.simpledrive.RobotRelativeDrive;
import frc.robot.commands.simpledrive.UpdateHeading;
import frc.robot.commands.step.DropInAmp;
import frc.robot.commands.step.LaunchSetDistance;
import frc.robot.commands.step.PassNote;
import frc.robot.commands.step.RunIntake;
import frc.robot.commands.step.LaunchSpit;
import frc.robot.commands.step.LaunchWithOdometry;
import frc.robot.commands.step.LobOntoField;
import frc.robot.commands.step.SimpleControl;
import frc.robot.commands.step.SourceIntake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive2022;
import frc.robot.subsystems.swerve.SwerveDrive2024;
import frc.robot.subsystems.swerve.SwerveDrive.Zone;
import frc.robot.subsystems.vision.VisionData;
import frc.robot.util.DriveDirection;
import frc.robot.util.FieldPose2024;
import frc.robot.util.RumbleManager;

public class RobotContainer {

  private Gamepad m_driver = new Gamepad(ControllerConstants.DriverPort, 10, 10);
  private Gamepad m_operator = new Gamepad(ControllerConstants.OperatorPort);
  public static Gamepad SimKeyboard;
  private Gamepad m_tester;
  // private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private SwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Vision m_vision = new Vision(() -> m_swerveDrive.getPose(), (data) -> updatePoseEstimator(data), () -> m_swerveDrive.getRobotSpeedMps());
  private Intake m_intake = new Intake();
  private Lift m_lift = new Lift();
  private Feeder m_feeder = new Feeder();
  private Launcher m_launcher = new Launcher();
  private PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);
  private RumbleManager m_rumbleManager = new RumbleManager(m_driver, m_operator, m_feeder, m_intake);
  private LightStrip m_leds = new LightStrip(() -> m_intake.hasNote(), () -> m_feeder.hasNote());
  private final SendableChooser<Command> m_pathPlannerChooser;

  private Supplier<Command> m_getSlowCommand = () -> new StartEndCommand(
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.SlowSpeedModifier),
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier)
  );


  private Supplier<LauncherTarget> m_getDefaultLauncherTarget = () -> {
    Zone currentZone = m_swerveDrive.getZone();
    Optional<LauncherTarget> launcherTargetsOptional = Optional.empty();
    switch(currentZone) {
      case NEAR:
        launcherTargetsOptional = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_lift.getCurrentHeightMeters(), m_swerveDrive.getDistanceToSpeakerMeters(), m_launcher.getAbsoluteTiltAngle(), TargetAngleMode.Lower);
        break;
      case MID:
        launcherTargetsOptional = LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_lift.getCurrentHeightMeters(), FieldPose2024.AmpPass.getDistanceFromLocation(m_swerveDrive.getPose()), Rotation2d.fromDegrees(45));
        break;
      case FAR:
        launcherTargetsOptional = LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_lift.getCurrentHeightMeters(), FieldPose2024.MidLinePass.getDistanceFromLocation(m_swerveDrive.getPose()), Rotation2d.fromDegrees(45));
        break;
    }
    if (launcherTargetsOptional.isEmpty()) {
      return new LauncherTarget(LauncherConstants.NoTargetRPM, 0.0, m_launcher.getAbsoluteTiltAngle().getDegrees(), m_lift.getCurrentHeightMeters());
    }
    var launcherTargets = launcherTargetsOptional.get();
    return launcherTargets; 
  };


  public RobotContainer() {
    m_PDH.setSwitchableChannel(false);
    m_swerveDrive.resetPose(FieldPose2024.TestStart.getCurrentAlliancePose());
    configureBindings();

    // Sets up the back camera with a pose offset to correct the pose
    // This generates the offset from the robot origin to the camera location
    m_vision.getCamera(CameraDirection.Back).setOffsetHandler(() -> {
      var launcherRotation = -(m_launcher.getAbsoluteTiltAngle().minus(LauncherConstants.MinAngle).getRadians());

      Translation3d LiftOffset = new Translation3d(-0.082, 0, 0.425);
      Translation3d LauncherOffset = new Translation3d(0.18, 0, 0.177);
      Translation3d StaticOffset = new Translation3d(-0.299, 0, 0.277);

      LiftOffset = LiftOffset.div(LiftOffset.getNorm());
      var liftheight = LiftOffset.times(m_lift.getCurrentHeightMeters());
      SmartDashboard.putNumberArray("CameraCalc/LiftHeight", new double[]{liftheight.getX(),liftheight.getY(),liftheight.getZ()});

      var rot = new Rotation3d(0, launcherRotation, 0);
      SmartDashboard.putNumber("CameraCalc/rot", launcherRotation);
      var rotatedLauncherVector = LauncherOffset.rotateBy(rot);
      SmartDashboard.putNumberArray("CameraCalc/LauncherVector", new double[]{rotatedLauncherVector.getX(),rotatedLauncherVector.getY(),rotatedLauncherVector.getZ()});

      var limelightlocation = rotatedLauncherVector.plus(liftheight).plus(StaticOffset);
      SmartDashboard.putNumberArray("CameraCalc/FinalVector", new double[]{limelightlocation.getX(),limelightlocation.getY(),limelightlocation.getZ()});

      var finalRotation = new Rotation3d(0, m_launcher.getAbsoluteTiltAngle().getRadians()-VisionConstants.RearCameraMountAngleRadians, 0);
      return new Pose3d(limelightlocation, finalRotation);
    });

    NamedCommands.registerCommand("launch", new LaunchWithOdometry(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_getDefaultLauncherTarget));
    NamedCommands.registerCommand("launchWithTimeout", new LaunchWithOdometry(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_getDefaultLauncherTarget).withTimeout(3.0));
    NamedCommands.registerCommand("intake", new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager));
    NamedCommands.registerCommand("intakeWait", new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager).withTimeout(0.25));
    NamedCommands.registerCommand("launchSpit", new LaunchSpit(m_intake, m_lift, m_feeder, m_launcher));
    // Build an auto chooser. This will use Commands.none() as the default option.
    m_pathPlannerChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
    SmartDashboard.putData("Auto Chooser", m_pathPlannerChooser);
  }

  private void configureBindings() {
    if (Robot.isSimulation()) {
      SimKeyboard = new Gamepad(ControllerConstants.SimKeyboardPort);

      //SimKeyboard.x().whileTrue(AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpPath")));
      SimKeyboard.x().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_feeder, m_swerveDrive, m_vision));
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
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher, m_operator, m_getDefaultLauncherTarget, () -> 
    m_intake.hasNote()));
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

    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, FieldPose2024.Speaker, 1.0)); // Align angle to amp (but allow translation)
    m_driver.b().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageRight, 1.0)); // Align angle to stage left (but allow translation)
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageLeft, 1.0)); // Align angle to stage right (but allow translation)
    m_driver.y().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.MidLinePass, LiftConstants.IntakeHeightMeters, Rotation2d.fromDegrees(45), m_getDefaultLauncherTarget, true, "MidLinePass"));  // Align angle to HP (but allow translation)
    // m_driver.y().whileTrue(new LobOntoFieldSetDistance(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.PassFromSource, 8.0, LiftConstants.SourceIntakeHeightHighMeters, LauncherConstants.SourceIntakeAngleHigh, m_getdefaultLauncherTarget, true));  // Align angle to HP (but allow translation)

    //find a way for the leftBumpper commands not override one another ~ jojo ;)
    m_driver.leftBumper().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.AmpPass, LiftConstants.IntakeHeightMeters, Rotation2d.fromDegrees(45), m_getDefaultLauncherTarget, false, "AmpPass"));
    m_driver.leftTrigger().whileTrue(new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager)); // Intake
    // m_driver.rightBumper().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_feeder, m_swerveDrive, m_vision)); // Amp score
    m_driver.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder)); // Amp score
    m_driver.rightTrigger() // Aim and launch at speaker 
      .whileTrue( 
        new LaunchWithOdometry(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_getDefaultLauncherTarget));

    m_driver.leftStick().whileTrue(m_getSlowCommand.get()); //
    m_driver.rightStick(); //
  }

  private void configureOperatorCommands() {
    m_operator.back(); // Enable Automation (?)
    m_operator.start(); // Disable Automation (?)

    m_operator.povUp().whileTrue(new PassNote(m_intake, m_lift, m_feeder, m_launcher)); // Reverse Intake (dumb)
    m_operator.povDown().whileTrue(new InstantCommand(() -> DefaultLauncherCommand.LauncherPreSpinEnabled = false).alongWith(new SimpleControl().flywheel(m_launcher, -0.05))); // Disable launcher prespin
    m_operator.povLeft().whileTrue(new SimpleControl().feeder(m_feeder, 0.3, 0)); // Position note for trap 
    m_operator.povRight().whileTrue(new InstantCommand(() -> DefaultLauncherCommand.LauncherPreSpinEnabled = true)); // Re-enable launcher prespin

    Function<Double, StartEndCommand> createGetHeightCommand = (Double height) -> new StartEndCommand(() -> m_lift.moveToHeight(height), () -> m_lift.setSpeed(0), m_lift);
    Function<Rotation2d, StartEndCommand> createGetTiltCommand = (Rotation2d angle) -> new StartEndCommand(() -> m_launcher.setTiltAngle(angle), () -> m_launcher.setTiltSpeed(0), m_launcher);
    m_operator.a().whileTrue(createGetHeightCommand.apply(LiftConstants.MinHeightMeters)); // Min height
    m_operator.b().whileTrue(createGetHeightCommand.apply(LiftConstants.AmpMeters)); // Amp Height
    m_operator.x().whileTrue(new SourceIntake(m_lift, m_feeder, m_launcher)); // HP Intake
    m_operator.y().whileTrue(
      createGetHeightCommand.apply(LiftConstants.MaxHeightMeters)
        .alongWith(createGetTiltCommand.apply(LauncherConstants.TrapAngle))
        .alongWith(m_getSlowCommand.get())); // Max height

    m_operator.leftBumper().whileTrue(new LaunchSetDistance(m_lift, m_launcher, m_feeder, m_intake, FieldPose2024.PodiumLaunch, LiftConstants.MaxHeightMeters, m_getDefaultLauncherTarget));
    m_operator.leftTrigger().whileTrue(new LaunchSetDistance(m_lift, m_launcher, m_feeder, m_intake, FieldPose2024.FenderLaunch, m_getDefaultLauncherTarget));
    m_operator.rightBumper().whileTrue(new SimpleControl().feeder(m_feeder, -0.1)); // Amp & Down Trap
    m_operator.rightTrigger().whileTrue(new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager)); // Intake (smart)

    m_operator.leftStick(); //
    m_operator.rightStick(); //

    // left stick y - manual angle tilt
    // right stick y - manual lift height
  }

  private void configureTesterCommands() {
    // m_tester.a().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(15)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    // m_tester.b().whileTrue(new StartEndCommand(() -> m_launcher.setTiltAngle(Rotation2d.fromDegrees(40)), () -> m_launcher.setTiltSpeed(0), m_launcher));
    // m_tester.x().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.2), () -> m_lift.setSpeed(0), m_lift));
    // m_tester.y().whileTrue(new StartEndCommand(() -> m_lift.moveToHeight(0.6), () -> m_lift.setSpeed(0), m_lift));
    // m_tester.leftBumper().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(2000), () -> m_launcher.setLauncherPower(0), m_launcher));
    // m_tester.leftTrigger().whileTrue(new StartEndCommand(() -> m_launcher.setLauncherRPM(5000), () -> m_launcher.setLauncherPower(0), m_launcher));
    
    //m_tester.rightTrigger().whileTrue(new DashboardLaunch(m_lift, m_launcher, m_feeder, m_intake));
    //m_tester.rightTrigger().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_feeder, m_swerveDrive, m_vision));
    // m_tester.rightTrigger().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_feeder, m_swerveDrive, m_vision));
    // m_tester.x().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.Note2, 0.15, Rotation2d.fromDegrees(45), m_getdefaultLauncherTarget, false));
    // m_tester.a().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.Note2, 0.15, Rotation2d.fromDegrees(10), m_getdefaultLauncherTarget, false));
    // m_tester.b().whileTrue(new SourceIntake(m_lift, m_feeder, m_launcher));
  }
  
  public Command getAutonomousCommand() {
    // return m_autoBuilder.createAutoCommand();
    return m_pathPlannerChooser.getSelected();
  }

  public void periodic() {
    // Enables Dashboard Numbers to be updated each loop
    DashboardNumber.checkAll();

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

    var drivePose = m_swerveDrive.getPose();
    var frontCameraPose = m_vision.getCamera(CameraDirection.Front).getMostRecentPose();
    frontCameraPose = frontCameraPose != null ? frontCameraPose : new Pose2d();
    var backCameraPose = m_vision.getCamera(CameraDirection.Back).getMostRecentPose();
    backCameraPose = backCameraPose != null ? backCameraPose : new Pose2d();

    double[] robotAndVision = {
      drivePose.getX(),
      drivePose.getY(),
      drivePose.getRotation().getDegrees(),
      frontCameraPose.getX(),
      frontCameraPose.getY(),
      frontCameraPose.getRotation().getDegrees(),
      backCameraPose.getX(),
      backCameraPose.getY(),
      backCameraPose.getRotation().getDegrees()
    };
    SmartDashboard.putNumberArray("Robot and Vision", robotAndVision);

    SmartDashboard.putNumber("Distance to Speaker (odometry)", m_swerveDrive.getDistanceToSpeakerMeters());
    SmartDashboard.putNumber("Distance to Speaker (ty)", LauncherModel.speakerAprilTagTyToBotCenterDistanceMeters(m_vision.getCamera(CameraDirection.Front).getTargetElevation(true)));

    // Doing these rumbles in this periodic function so they trigger for regardless of what driver or operator command is being run
    if (!DriverStation.isTeleopEnabled()){
      m_rumbleManager.disableRumble();
    } 
  }

  public void autoAndTeleopInit(boolean isAuto) {
    DefaultLauncherCommand.LauncherPreSpinEnabled = true;
    m_lift.changeNeutralMode(NeutralModeValue.Brake);
  }

  public void delayedDisableInit() {
    m_lift.changeNeutralMode(NeutralModeValue.Coast);
  }

  public synchronized void updatePoseEstimator(VisionData data) {
    
    m_swerveDrive.addVisionMeasurement(data.getPose2d(), data.getTimestamp(), data.getDevation());
  }
}


