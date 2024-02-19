// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/**
 * A class for controlling the rollers (intake, feeder, and launcher) in a simple way. Each respective subsystem can optionally be added via chaining:
 * 
 * <code>
 * new SimpleRollerControl().intake(m_intake, 0.5).feeder(m_feeder, 0.3);
 * </code
 */
public class SimpleRollerControl extends Command {
  private Intake m_intake;
  private Launcher m_launcher;
  private Feeder m_feeder;

  private double m_intakePower = Double.NaN;
  private double m_feederPower = Double.NaN;
  private double m_launcherPower = Double.NaN;

  /**
   * Adds the intake to the roller controller, running it at the set power
   * @param intake the intake (will be added as a requirement of the command)
   * @param intakePower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleRollerControl intake(Intake intake, double intakePower) {
    m_intake = intake;
    m_intakePower = intakePower;
    addRequirements(m_intake);
    return this;
  }

  /**
   * Adds the feeder to the roller controller, running it at the set power
   * @param feeder the feeder (will be added as a requirement of the command)
   * @param feederPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleRollerControl feeder(Feeder feeder, double feederPower) {
    m_feeder = feeder;
    m_feederPower = feederPower;
    addRequirements(m_feeder);
    return this;
  }

  /**
   * Adds the launcher to the roller controller, running it at the set power
   * @param launcher the launcher (will be added as a requirement of the command)
   * @param launcherPower the duty cycle power [-1.0, 1.0] to run at
   */
  public SimpleRollerControl launcher(Launcher launcher, double launcherPower) {
    m_launcher = launcher;
    m_launcherPower = launcherPower;
    addRequirements(m_launcher);
    return this;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only need to start a mechanism if the command was configured to use it

    if (Double.isFinite(m_intakePower)) {
      m_intake.setIntakePower(m_intakePower);
    }

    if (Double.isFinite(m_feederPower)) {
      m_feeder.setFeederPower(m_feederPower);
    }

    if (Double.isFinite(m_launcherPower)) {
      m_launcher.setLauncherPower(m_launcherPower);
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

    if (Double.isFinite(m_launcherPower)) {
      m_launcher.setLauncherPower(0.0);
    }
  }
}
