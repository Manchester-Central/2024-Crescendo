// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class DefaultFeederCommand extends Command {
  private Feeder m_feeder;
  private Gamepad m_tester;
  private Supplier<Boolean> m_intakeHasNoteSupplier;
  /** Creates a new DefaultFeederCommand. */
  public DefaultFeederCommand(Feeder feeder, Gamepad operator, Supplier<Boolean> intakeHasNoteSupplier) {
    m_feeder = feeder;
    m_tester = operator;
    m_intakeHasNoteSupplier = intakeHasNoteSupplier;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (RobotContainer.PreSpinEnabled) {
      m_feeder.grabAndHoldPiece(m_intakeHasNoteSupplier.get() ? 0.35 : 0.0); // Position the note still if we have one
    } else {
      m_feeder.setFeederPower(0);
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
