// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.step;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.launcher.Launcher;

// TODO: Implement actual control logic
public class BattleCryAmp extends Command {
  private Lift m_lift;
  private Launcher m_launcher;
  private Feeder m_feeder;

  /** Creates a new DropInAmp. */
  public BattleCryAmp(Lift lift, Launcher launcher, Feeder feeder) {
    m_lift = lift;
    m_launcher = launcher;
    m_feeder = feeder;
    addRequirements(lift, launcher, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.moveToHeight(LiftConstants.BattleCryAmpMeters);
    m_launcher.setTiltAngle(LauncherConstants.BattleCryAmpAngle);
    m_feeder.setFeederPower(0.5);
    m_launcher.setLauncherPower(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setFeederPower(0);
    m_launcher.setLauncherPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
