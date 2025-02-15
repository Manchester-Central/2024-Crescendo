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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.defaults.DefaultLightStripCommand;
import frc.robot.commands.defaults.DefaultVisionCommand;
import frc.robot.commands.simpledrive.DriverRelativeDrive;
import frc.robot.commands.simpledrive.DriverRelativeSetAngleDrive;
import frc.robot.commands.simpledrive.RobotRelativeDrive;
import frc.robot.commands.simpledrive.RobotRelativeSetAngleDrive;
import frc.robot.commands.simpledrive.UpdateHeading;
import frc.robot.commands.step.BattleCryAmp;
import frc.robot.commands.step.DemoLaunch;
import frc.robot.commands.step.DropInAmp;
import frc.robot.commands.step.DropInTrap;
import frc.robot.commands.step.LaunchSetDistance;
import frc.robot.commands.step.PassNote;
import frc.robot.commands.step.RunIntake;
import frc.robot.commands.step.RunIntakeUntilNote;
import frc.robot.commands.step.LaunchSpit;
import frc.robot.commands.step.LaunchWithOdometry;
import frc.robot.commands.step.LaunchWithOdometryAndVision;
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
import frc.robot.util.Pose2dUtil;
import frc.robot.util.RumbleManager;

public class RobotContainer {

  public static boolean PreSpinEnabled = true;
  public boolean m_isPoseUpdateEnabled = true;

  private Gamepad m_driver = new Gamepad(ControllerConstants.DriverPort, 50, 50);
  private Gamepad m_operator = new Gamepad(ControllerConstants.OperatorPort);
  public static Gamepad SimKeyboard;
  private Gamepad m_tester;
  // private final AutoBuilder m_autoBuilder = new AutoBuilder();

  private SwerveDrive m_swerveDrive = Constants.Use2022Robot 
    ? SwerveDrive2022.createSwerveDrive() 
    : SwerveDrive2024.createSwerveDrive();

  private Vision m_vision = new Vision(() -> m_swerveDrive.getPose(), (data) -> updatePoseEstimator(data), (targetPose) -> updateDemoTarget(targetPose), () -> m_swerveDrive.getRobotSpeedMps(), () -> m_swerveDrive.getRobotRotationSpeedRadsPerSec());
  private Intake m_intake = new Intake();
  private Lift m_lift = new Lift();
  private Feeder m_feeder = new Feeder();
  private Launcher m_launcher = new Launcher();
  private PowerDistribution m_PDH;
  private RumbleManager m_rumbleManager = new RumbleManager(m_driver, m_operator, m_feeder, m_intake);
  private LightStrip m_leds = new LightStrip();
  private final SendableChooser<Command> m_pathPlannerChooser;

  private Supplier<Command> m_getSlowCommand = () -> new StartEndCommand(
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.SlowSpeedModifier),
      () -> m_swerveDrive.updateSpeedModifier(SwerveConstants2024.DefaultSpeedModifier)
  );

  private boolean m_isAutomationEnabled = true;
  private Trigger m_isAutomationEnabledTrigger = new Trigger(() -> m_isAutomationEnabled);

  private Trigger m_isDemoModeTrigger = new Trigger(() -> DriverStation.isTest());
  private Trigger m_hasDemoTargetTrigger = new Trigger(() -> m_vision.getCamera(CameraDirection.Front).hasTarget());
  private Pose3d m_demoTargetPose = new Pose3d();

  private Supplier<LauncherTarget> m_getDefaultLauncherTarget = () -> {
    Zone currentZone = m_swerveDrive.getZone();
    Optional<LauncherTarget> launcherTargetsOptional = Optional.empty();
    switch(currentZone) {
      case NEAR:
        launcherTargetsOptional = LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, m_lift.getCurrentHeightMeters(), m_swerveDrive.getDistanceToSpeakerMeters(), m_launcher.getEncoderTiltAngle(), TargetAngleMode.Lower);
        break;
      case MID:
        launcherTargetsOptional = LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_lift.getCurrentHeightMeters(), FieldPose2024.AmpPass.getDistanceFromLocation(m_swerveDrive.getPose()), Rotation2d.fromDegrees(45));
        break;
      case FAR:
        launcherTargetsOptional = LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_lift.getCurrentHeightMeters(), FieldPose2024.MidLinePass.getDistanceFromLocation(m_swerveDrive.getPose()), Rotation2d.fromDegrees(45));
        break;
      case DEMO: 
        launcherTargetsOptional = LauncherModel.getLauncherTarget(m_demoTargetPose.getZ(), m_lift.getCurrentHeightMeters(), Pose2dUtil.getDistanceMeters(m_demoTargetPose.toPose2d(), m_swerveDrive.getPose()), m_launcher.getEncoderTiltAngle(), TargetAngleMode.Lower);
        break;
    }
    if (launcherTargetsOptional.isEmpty()) {
      return new LauncherTarget(LauncherConstants.NoTargetRPM, 0.0, m_launcher.getEncoderTiltAngle().getDegrees(), m_lift.getCurrentHeightMeters());
    }
    var launcherTargets = launcherTargetsOptional.get();
    return launcherTargets; 
  };


  public RobotContainer(PowerDistribution pdh) {
    m_PDH = pdh;
    m_PDH.setSwitchableChannel(true);
    m_swerveDrive.resetPose(FieldPose2024.TestStart.getCurrentAlliancePose());
    configureBindings();

    // Sets up the back camera with a pose offset to correct the pose
    // This generates the offset from the robot origin to the camera location
    m_vision.getCamera(CameraDirection.Back).setOffsetHandler(() -> {
      var launcherRotation = -(m_launcher.getEncoderTiltAngle().minus(LauncherConstants.MinAngle).getRadians());

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

      var finalRotation = new Rotation3d(0, m_launcher.getEncoderTiltAngle().getRadians()-VisionConstants.RearCameraMountAngleRadians, 0);
      return new Pose3d(limelightlocation, finalRotation);
    });

    NamedCommands.registerCommand("launch", new LaunchWithOdometryAndVision(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_vision, m_leds, m_getDefaultLauncherTarget, () -> true));
    NamedCommands.registerCommand("launchWithTimeout", new LaunchWithOdometryAndVision(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_vision, m_leds, m_getDefaultLauncherTarget, () -> true).withTimeout(3.0));
    NamedCommands.registerCommand("intake", new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager));
    NamedCommands.registerCommand("intakeWait", new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager).withTimeout(0.25));
    NamedCommands.registerCommand("launchSpit", new LaunchSpit(m_intake, m_lift, m_feeder, m_launcher));
    NamedCommands.registerCommand("disableOdometryUpdates", new InstantCommand(() -> m_isPoseUpdateEnabled = false));
    NamedCommands.registerCommand("enableOdometryUpdates", new InstantCommand(() -> m_isPoseUpdateEnabled = true));
    NamedCommands.registerCommand("intakeUntilNote", new RunIntakeUntilNote(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager));
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    m_pathPlannerChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
    SmartDashboard.putData("Auto Chooser", m_pathPlannerChooser);
  }

  private void configureBindings() {
    if (Robot.isSimulation()) {
      // SimKeyboard should only be used for faking values (such as the note sensors while simulating)
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
    m_vision.setDefaultCommand(new DefaultVisionCommand(m_vision));
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_driver, m_swerveDrive));
    // m_swerveDrive.setDefaultCommand(robotRelativeDrive);
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake, () -> m_feeder.hasNote()));
    m_lift.setDefaultCommand(new DefaultLiftCommand(m_lift, m_operator));
    m_launcher.setDefaultCommand(new DefaultLauncherCommand(m_launcher, m_operator, m_getDefaultLauncherTarget, () -> 
    m_feeder.hasNote()));
    m_feeder.setDefaultCommand(new DefaultFeederCommand(m_feeder, m_tester, () -> m_intake.hasNote()));
    m_leds.setDefaultCommand(new DefaultLightStripCommand(m_leds, () -> m_intake.hasNote(), () -> m_feeder.hasNote(), ()-> m_operator.getHID().getStartButton()));
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
    m_driver.b().whileTrue(new RobotRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageRight, 1.0, true)); // Align angle to stage left (but allow translation)
    m_driver.x().whileTrue(new RobotRelativeSetAngleDrive(m_driver, m_swerveDrive, DriveDirection.FacingStageLeft, 1.0, true)); // Align angle to stage right (but allow translation)
    m_driver.y().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.MidLinePass, LiftConstants.IntakeHeightMeters, Rotation2d.fromDegrees(45), m_getDefaultLauncherTarget, true, "MidLinePass"));  // Align angle to HP (but allow translation)
    // m_driver.y().whileTrue(new LobOntoFieldSetDistance(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.PassFromSource, 8.0, LiftConstants.SourceIntakeHeightHighMeters, LauncherConstants.SourceIntakeAngleHigh, m_getdefaultLauncherTarget, true));  // Align angle to HP (but allow translation)

    //find a way for the leftBumpper commands not override one another ~ jojo ;)
    m_driver.leftBumper().whileTrue(new LobOntoField(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, FieldPose2024.AmpPass, LiftConstants.IntakeHeightMeters, Rotation2d.fromDegrees(45), m_getDefaultLauncherTarget, false, "AmpPass"));
    m_driver.leftTrigger().whileTrue(new RunIntake(m_intake, m_lift, m_feeder, m_launcher, m_getDefaultLauncherTarget, m_rumbleManager)); // Intake
    // m_driver.rightBumper().whileTrue(new FireIntoAmp(m_lift, m_launcher, m_feeder, m_swerveDrive, m_vision)); // Amp score
    m_driver.rightBumper().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder)); // Amp score
    // m_driver.rightTrigger().and(m_isOdometryAndLaunchModeEnabledTrigger.negate()) // Aim and launch at speaker 
    //   .whileTrue( 
    //     new LaunchWithOdometry(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_getDefaultLauncherTarget));
    m_driver.rightTrigger().and(m_isDemoModeTrigger.negate()) // Aim and launch at speaker 
      .whileTrue( 
        new LaunchWithOdometryAndVision(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_vision, m_leds, m_getDefaultLauncherTarget, () -> true));
    m_driver.rightTrigger().and(m_isDemoModeTrigger).and(m_hasDemoTargetTrigger) // Aim and launch at demo target 
      .whileTrue( 
        new DemoLaunch(m_lift, m_launcher, m_feeder, m_swerveDrive, m_driver, m_intake, m_leds, () -> m_demoTargetPose, m_getDefaultLauncherTarget));

    m_driver.leftStick().whileTrue(m_getSlowCommand.get()); //
    m_driver.rightStick(); //
  }

  private void configureOperatorCommands() {
    //m_operator.back().onTrue(new InstantCommand(() -> m_isAutomationEnabled = false)); // Disable Automation (?)
    //m_operator.start().onTrue(new InstantCommand(() -> m_isAutomationEnabled = true)); // Enable Automation (?)

    // start is used for LED's 

    m_operator.povUp().whileTrue(new PassNote(m_intake, m_lift, m_feeder, m_launcher)); // Reverse Intake (dumb)
    m_operator.povDown().whileTrue(new InstantCommand(() -> PreSpinEnabled = false).alongWith(new SimpleControl().flywheel(m_launcher, -0.05))); // Disable launcher prespin
    m_operator.povLeft().whileTrue(new SimpleControl().feeder(m_feeder, 0.3, 0)); // Position note for trap 
    m_operator.povRight().whileTrue(new InstantCommand(() -> PreSpinEnabled = true)); // Re-enable launcher prespin

    Function<Double, StartEndCommand> createGetHeightCommand = (Double height) -> new StartEndCommand(() -> m_lift.moveToHeight(height), () -> m_lift.setSpeed(0), m_lift);
    Function<Rotation2d, StartEndCommand> createGetTiltCommand = (Rotation2d angle) -> new StartEndCommand(() -> m_launcher.setTiltAngle(angle), () -> m_launcher.setTiltSpeed(0), m_launcher);
    m_operator.a().whileTrue(createGetHeightCommand.apply(LiftConstants.MinHeightMeters)); // Min height
    m_operator.b().whileTrue(new DropInAmp(m_lift, m_launcher, m_feeder)); // Amp Height
    m_operator.x().whileTrue(new SourceIntake(m_lift, m_feeder, m_launcher)); // HP Intake
    m_operator.y().whileTrue(
      createGetHeightCommand.apply(LiftConstants.MaxHeightMeters)
        .alongWith(createGetTiltCommand.apply(LauncherConstants.TrapAngle))
        .alongWith(m_getSlowCommand.get())); // Max height

    m_operator.leftBumper().whileTrue(new LaunchSetDistance(m_lift, m_launcher, m_feeder, m_intake, FieldPose2024.PodiumLaunch, LiftConstants.MaxHeightMeters, m_getDefaultLauncherTarget));
    m_operator.leftTrigger().whileTrue(new LaunchSetDistance(m_lift, m_launcher, m_feeder, m_intake, FieldPose2024.FenderLaunch, m_getDefaultLauncherTarget));
    m_operator.rightBumper().and(m_isAutomationEnabledTrigger.negate()).whileTrue(new SimpleControl().feeder(m_feeder, -0.1)); // Amp & Down Trap
    m_operator.rightBumper().and(m_isAutomationEnabledTrigger).whileTrue(new DropInTrap(m_lift, m_launcher, m_feeder));
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

    // Chaosboard expects: [intakePower, liftHeightMeters, launcherAngleDegrees, feederPower, launcherRpm, atFeederPrimary, atFeederSecondary, atFeederSecondary, atIntake]
    double[] RobotState = {
      m_intake.getCurrentIntakePower(),
      m_lift.getCurrentHeightMeters(),
      m_launcher.getEncoderTiltAngle().getDegrees(),
      m_feeder.getCurrentFeederPower(),
      m_launcher.getLeftLauncherRPM(),
      m_feeder.hasNoteAtPrimary() ? 1 : 0,
      m_feeder.hasNoteAtSecondary() ? 1 : 0,
      m_feeder.hasNoteAtTertiary() ? 1 : 0,
      m_intake.hasNote() ? 1 : 0,
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

    SmartDashboard.putNumber("Robot Angular Velocity", m_swerveDrive.getRobotRotationSpeedRadsPerSec());

    SmartDashboard.putNumber("Distance BotCenter to SpeakerAprilTag (odometry)", m_swerveDrive.getDistanceToSpeakerMeters());
    // SmartDashboard.putNumber("Distance Limelight to SpeakerAprilTag (odometry)", m_swerveDrive.getDistanceToSpeakerMeters() - 0.177306);
    SmartDashboard.putNumber("Distance BotCenter to SpeakerOpening (ty)", LauncherModel.speakerOpeningToBotCenterDistanceMetersByTY(m_vision.getCamera(CameraDirection.Front).getTargetElevation(true)));
    // SmartDashboard.putNumber("Distance BotCenter to SpeakerAprilTag (ty)", LauncherModel.speakerAprilTagToBotCenterDistanceMetersByTY(m_vision.getCamera(CameraDirection.Front).getTargetElevation(true)));
    // SmartDashboard.putNumber("Distance Limelight to SpeakerAprilTag (ty)", LauncherModel.speakerAprilTagToLimelightDistanceMetersByTY(m_vision.getCamera(CameraDirection.Front).getTargetElevation(true)));
    // Doing these rumbles in this periodic function so they trigger for regardless of what driver or operator command is being run
    if (!DriverStation.isTeleopEnabled()){
      m_rumbleManager.disableRumble();
    } 
  }

  public void autoAndTeleopInit(boolean isAuto) {
    PreSpinEnabled = true;
    m_isPoseUpdateEnabled = true;
    m_lift.changeNeutralMode(NeutralModeValue.Brake);
  }

  public void demoInit(){
    PreSpinEnabled = false;
    m_isPoseUpdateEnabled = false;
    m_lift.changeNeutralMode(NeutralModeValue.Brake);
  }

  public void delayedDisableInit() {
    m_lift.changeNeutralMode(NeutralModeValue.Coast);
  }

  public synchronized void updatePoseEstimator(VisionData data) {
    var pose = data.getPose2d();
    if (!m_isPoseUpdateEnabled) {
      return;
    }
    if (pose == null || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY()) || !Double.isFinite(pose.getRotation().getDegrees())){
      return;
    }

    m_swerveDrive.addVisionMeasurement(pose, data.getTimestamp(), data.getDevation());
  }

  public synchronized void updateDemoTarget(Pose3d targetPose) {
    m_demoTargetPose = targetPose;
  }
}


