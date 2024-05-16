// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.LightStrip.Color;

public class DefaultLightStripCommand extends Command {
  private LightStrip m_lightStrip;
  private Supplier<Boolean> m_intakeSupplier;
  private Supplier<Boolean> m_feederSupplier;
  private Supplier<Boolean> m_operatorSupplier;

  /** Creates a new DefaultLightStripCommand. */
  public DefaultLightStripCommand(LightStrip lightStrip, Supplier<Boolean> intakeHasNote, Supplier<Boolean> feederHasNote, Supplier<Boolean> operatorSupplier) {
    m_lightStrip = lightStrip;
    m_intakeSupplier = intakeHasNote;
    m_feederSupplier = feederHasNote;
    m_operatorSupplier = operatorSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Logic for an enabled robot
    if (m_intakeSupplier.get() == true) {
      m_lightStrip.setSingleColor(Color.YELLOW);
    } else if (m_feederSupplier.get() == true && m_intakeSupplier.get() == false) {
      m_lightStrip.setSingleColor(Color.GREEN);
    } else if(m_operatorSupplier.get()){ 
      m_lightStrip.setSingleColor(Color.WHITE);
    } else {
      m_lightStrip.setSingleColor(DriverStation.getAlliance().get() == Alliance.Blue ? Color.BLUE : Color.RED);
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
