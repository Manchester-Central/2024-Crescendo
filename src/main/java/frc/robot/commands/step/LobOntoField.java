// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.util.DashboardNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherSpeeds;
import frc.robot.subsystems.launcher.LauncherTarget;
import frc.robot.util.FieldPose2024;

public class LobOntoField extends BaseLaunch {
  private final double kSwerveAngleTolerance = 2.0;
  private BaseSwerveDrive m_swerveDrive;
  private Gamepad m_driver;
  private FieldPose2024 m_targetPose;
  private double m_liftHeight;
  private Rotation2d m_launchAngle;
  private boolean m_isSourceIntake;
  private DashboardNumber m_launchDegreesAdjuster;

  /** Creates a new Lanch Partay. */
  public LobOntoField(
      Lift lift,
      Launcher launcher,
      Feeder feeder,
      BaseSwerveDrive swerveDrive,
      Gamepad driver,
      Intake intake,
      FieldPose2024 targetPose,
      double liftHeight,
      Rotation2d launchAngle,
      Supplier<LauncherTarget> getDefaultLauncherTarget,
      boolean isSourceIntake,
      String name
  ) {
    super(lift, launcher, feeder, intake, getDefaultLauncherTarget);
    m_swerveDrive = swerveDrive;
    m_driver = driver;
    m_targetPose = targetPose;
    m_liftHeight = liftHeight;
    m_launchAngle = launchAngle;
    m_isSourceIntake = isSourceIntake;
    m_launchDegreesAdjuster = new DashboardNumber("LobOntoField/"+name+" Tilt Angle Degrees", launchAngle.getDegrees(), true, (newValue) -> m_launchAngle = Rotation2d.fromDegrees(newValue));

    addRequirements(lift, launcher, feeder, swerveDrive);
  }

  @Override
  protected double getFeederLaunchSpeed() {
    return 1.0;
  }

  @Override
  protected double getTrapLaunchSpeed() {
    return m_isSourceIntake ? -1.0 : 1.0;
  }

  @Override
  public void initialize() {
    m_swerveDrive.resetPids();
    super.initialize();
  }

  @Override
  public void execute() {
    var targetAngle = getTargetAngle();
    m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 1.0);
    super.execute();
  }

  private Rotation2d getTargetAngle(){
    return m_targetPose.angleFrom(m_swerveDrive.getPose());
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
    super.end(interrupted);
  }

  @Override
  protected Optional<LauncherTarget> getTargets() {
    var currentPose = m_swerveDrive.getPose();
    double distanceToTargetMeters = Math.abs(m_targetPose.getCurrentAlliancePose().getTranslation().getDistance(currentPose.getTranslation()));
    return LauncherModel.getLauncherTargetWithAngle(LauncherHeightTarget.Floor, m_liftHeight, distanceToTargetMeters, m_launchAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  protected boolean isClearToLaunch() {
    // TODO - handle logic better for when shooting on the fly
    return Math.abs(getDriveAngleErrorDegrees()) < kSwerveAngleTolerance;
  }

  private double getDriveAngleErrorDegrees() {
    return getTargetAngle().minus(m_swerveDrive.getOdometryRotation()).getDegrees();
  }

  @Override
  protected List<String> getLaunchErrors() {
    var errors = super.getLaunchErrors();
    errors.add(formatError("Drive Angle", getDriveAngleErrorDegrees()));
    return errors;
  }
}
