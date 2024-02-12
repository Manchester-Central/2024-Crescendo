// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;

public class DefaultLiftCommand extends Command {
  private Lift m_lift;

  /** Creates a new DefaultLiftCommand. */
  public DefaultLiftCommand(Lift lift) {
    m_lift = lift;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!m_lift.hasSeenBottom()){
      m_lift.setSpeed(-0.2);
    } else {
       m_lift.moveToHeight(LiftConstants.DefaultHoldMeters);
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
