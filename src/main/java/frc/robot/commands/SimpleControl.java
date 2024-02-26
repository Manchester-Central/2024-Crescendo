// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;

/**
 * A class for controlling the mechanisms (intake, feeder, and flywheels) in a simple way. Each respective subsystem can optionally be added via chaining:
 * 
 * <code>
 * new SimpleControl().intake(m_intake, 0.5).feeder(m_feeder, 0.3);
 * </code
 */
public class SimpleControl extends Command {
  private Intake m_intake = null;
  private Launcher m_launcher = null;
  private Feeder m_feeder = null;
  private Lift m_lift = null;

  private double m_intakePower = Double.NaN;
  private double m_feederPower = Double.NaN;
  private double m_trapPower = Double.NaN;
  private double m_flywheelPower = Double.NaN;
  private double m_tiltPower = Double.NaN;
  private double m_liftPower = Double.NaN;

  /**
   * Adds the intake to the simple controller, running it at the set power
   * @param intake the intake (will be added as a requirement of the command)
   * @param intakePower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleControl intake(Intake intake, double intakePower) {
    m_intakePower = intakePower;
    if (m_intake == null) {
      m_intake = intake;
      addRequirements(m_intake);
    }
    return this;
  }

  /**
   * Adds the feeder to the simple controller, running it at the set power
   * @param feeder the feeder (will be added as a requirement of the command)
   * @param feederPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleControl feeder(Feeder feeder, double feederPower) {
    return feeder(feeder, feederPower, feederPower);
  }

  /**
   * Adds the feeder to the simple controller, running it at the set power
   * @param feeder the feeder (will be added as a requirement of the command)
   * @param feederPower the duty cycle power [-1.0, 1.0] to run the main feeder rollers at
   * @param trapPower the duty cycle power [-1.0, 1.0] to run at the trap roller at
   */
  public SimpleControl feeder(Feeder feeder, double feederPower, double trapPower) {
    m_feederPower = feederPower;
    m_trapPower = trapPower;
    if (m_feeder == null) {
      m_feeder = feeder;
      addRequirements(m_feeder);
    }
    return this;
  }

  /**
   * Adds the launcher's flywheels to the simple controller, running it at the set power
   * @param launcher the launcher (will be added as a requirement of the command)
   * @param flywheelPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleControl flywheel(Launcher launcher, double flywheelPower) {
    m_flywheelPower = flywheelPower;
    if (m_launcher == null) {
      m_launcher = launcher;
      addRequirements(m_launcher);
    }
    return this;
  }

  /**
   * Adds the launcher's tilt to the simple controller, running it at the set power
   * @param launcher the launcher (will be added as a requirement of the command)
   * @param flywheelPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleControl tilt(Launcher launcher, double tiltPower) {
    // Ensure we don't allow the auto to tilt faster than the operator (in open loop mode)
    m_tiltPower = MathUtil.clamp(tiltPower, -DefaultLauncherCommand.MaxTiltSpeed, DefaultLauncherCommand.MaxTiltSpeed);
    if (m_launcher == null) {
      m_launcher = launcher;
      addRequirements(m_launcher);
    }
    return this;
  }

  /**
   * Adds the lift to the simple controller, running it at the set power
   * @param launcher the launcher (will be added as a requirement of the command)
   * @param flywheelPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleControl lift(Lift lift, double liftPower) {
    m_liftPower = liftPower;
    if (m_lift == null) {
      m_lift = lift;
      addRequirements(m_lift);
    }
    return this;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only need to start a mechanism if the command was configured to use it

    if (Double.isFinite(m_intakePower)) {
      m_intake.setIntakePower(m_intakePower);
    }

    if (Double.isFinite(m_feederPower) && Double.isFinite(m_trapPower)) {
      m_feeder.setFeederPower(m_feederPower, m_trapPower);
    }

    if (Double.isFinite(m_flywheelPower)) {
      m_launcher.setLauncherPower(m_flywheelPower);
    }

    if (Double.isFinite(m_tiltPower)) {
      m_launcher.setTiltSpeed(m_tiltPower);
    }

    if (Double.isFinite(m_liftPower)) {
      m_lift.setSpeed(m_liftPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Only need to stop a mechanism if the command was configured to use it

    if (Double.isFinite(m_intakePower)) {
      m_intake.setIntakePower(0.0);
    }

    if (Double.isFinite(m_feederPower)) {
      m_feeder.setFeederPower(0.0);
    }

    if (Double.isFinite(m_flywheelPower)) {
      m_launcher.setLauncherPower(0.0);
    }

    if (Double.isFinite(m_tiltPower)) {
      m_launcher.setTiltSpeed(0.0);
    }

    if (Double.isFinite(m_liftPower)) {
      m_lift.setSpeed(0.0);
    }
  }

  public static SimpleControl createAutoCommand(ParsedCommand parsedCommand, Intake intake, Feeder feeder, Launcher launcher, Lift lift) {
    var simpleControl = new SimpleControl();

    var intakePowerArg = parsedCommand.getArgument("intakePower");
    if (intakePowerArg != null) {
      simpleControl.intake(intake, AutoUtil.ParseDouble(intakePowerArg, 0));
    }

    var feederPowerArg = parsedCommand.getArgument("feederPower");
    if (feederPowerArg != null) {
      simpleControl.feeder(feeder, AutoUtil.ParseDouble(feederPowerArg, 0));
    }

    var flywheelPowerArg = parsedCommand.getArgument("flywheelPower");
    if (flywheelPowerArg != null) {
      simpleControl.flywheel(launcher, AutoUtil.ParseDouble(flywheelPowerArg, 0));
    }

    var tiltPowerArg = parsedCommand.getArgument("tiltPower");
    if (tiltPowerArg != null) {
      simpleControl.tilt(launcher, AutoUtil.ParseDouble(tiltPowerArg, 0));
    }
  
    var liftPowerArg = parsedCommand.getArgument("liftPower");
    if (liftPowerArg != null) {
      simpleControl.lift(lift, AutoUtil.ParseDouble(liftPowerArg, 0));
    }

    return simpleControl;
  }
}
