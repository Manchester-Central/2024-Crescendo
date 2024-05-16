// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.simpledrive;

import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.DriveDirection;
import frc.robot.util.FieldPose2024;

public class RobotRelativeSetAngleDrive extends Command {
  private Gamepad m_driver;
  private BaseSwerveDrive m_swerveDrive;
  double m_magnitude;
  Supplier<Rotation2d> m_rotationSupplier; // A supplier is needed in case a DriveDirection is used - we probably won't know our alliance on start up
  private boolean m_isTranslationInverted = false;

  public RobotRelativeSetAngleDrive(Gamepad driver, BaseSwerveDrive swerveDrive, Supplier<Rotation2d> rotationSupplier, double magnitude, boolean isTranslationInverted) {
    m_driver = driver;
    m_swerveDrive = swerveDrive;
    m_magnitude = magnitude;
    m_rotationSupplier = rotationSupplier;
    m_isTranslationInverted = isTranslationInverted;

    addRequirements(m_swerveDrive);
  }

  public RobotRelativeSetAngleDrive(Gamepad driver, BaseSwerveDrive swerveDrive, Rotation2d angle, double magnitude, boolean isTranslationInverted) {
    this(driver, swerveDrive, () -> angle, magnitude, isTranslationInverted);
  }

  public RobotRelativeSetAngleDrive(Gamepad driver, BaseSwerveDrive swerveDrive, DriveDirection direction, double magnitude, boolean isTranslationInverted) {
    this(driver, swerveDrive, () -> direction.getAllianceAngle(), magnitude, isTranslationInverted);
  }

  public RobotRelativeSetAngleDrive(Gamepad driver, BaseSwerveDrive swerveDrive, FieldPose2024 fieldpose, double magnitude, boolean isTranslationInverted) {
    this(driver, swerveDrive, () -> fieldpose.angleFrom(swerveDrive.getPose()), magnitude, isTranslationInverted);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driverModeInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var forwardSpeed = m_driver.getSlewLeftY();
    var sidewaySpeed = -m_driver.getSlewLeftX();
    if(m_isTranslationInverted){
      forwardSpeed = -forwardSpeed;
      sidewaySpeed = -sidewaySpeed;
    }
    m_swerveDrive.moveRobotRelativeAngle(forwardSpeed, sidewaySpeed, m_rotationSupplier.get(), m_magnitude);
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
