// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;

public class DefaultLiftCommand extends Command {
  private Lift m_lift;
  private Gamepad m_operator;

  /** Creates a new DefaultLiftCommand. */
  public DefaultLiftCommand(Lift lift, Gamepad operator) {
    m_lift = lift;
    m_operator = operator;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!m_lift.hasSeenBottom()){
      m_lift.setSpeed(-0.1);
    } else {
      // m_lift.moveToHeight(LiftConstants.DefaultHoldMeters);
      m_lift.setSpeed(m_operator.getRightY() * 0.8);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
