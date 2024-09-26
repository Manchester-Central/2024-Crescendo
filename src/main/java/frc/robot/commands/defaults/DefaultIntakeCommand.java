// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends Command {
  private Intake m_intake;
  private Supplier<Boolean> m_feederHasNoteSupplier;
  /** Creates a new DefaultIntakeCommand. */
  public DefaultIntakeCommand(Intake intake, Supplier<Boolean> feederHasNote) {
    m_intake = intake;
    m_feederHasNoteSupplier = feederHasNote;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_intake.hasNote() && !m_feederHasNoteSupplier.get()){
      m_intake.setIntakePower(1.0);
    }
    else
    {
      m_intake.setIntakePower(0.0);
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
