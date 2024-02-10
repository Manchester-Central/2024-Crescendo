package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveToLocation;
import frc.robot.subsystems.Vision;

public class tracker extends Command {
    private BaseSwerveDrive m_swerveDrive;
    private Vision m_vision;

    public tracker(BaseSwerveDrive swerveDrive, Vision vision){
        m_swerveDrive = swerveDrive;
        m_vision = vision;
        addRequirements(swerveDrive, vision);
    }

    public void initialize() {
		
	}

	

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
        Double tx = m_vision.noteDetection();
        if(tx == null) return;
        Rotation2d noteAngle = m_swerveDrive.getOdometryRotation().rotateBy( Rotation2d.fromDegrees(-tx));
        m_swerveDrive.moveFieldRelativeAngle(0, 0, noteAngle, 0.2);
	}

	/**
	 * The action to take when the command ends. Called when either the command finishes normally, or
	 * when it interrupted/canceled.
	 *
	 * <p>Do not schedule commands here that share requirements with this command. Use {@link
	 * #andThen(Command...)} instead.
	 *
	 * @param interrupted whether the command was interrupted/canceled
	 */
	public void end(boolean interrupted) {
	
	}

	/**
	 * Whether the command has finished. Once a command finishes, the scheduler will call its end()
	 * method and un-schedule it.
	 *
	 * @return whether the command has finished.
	 */
	public boolean isFinished(){
    return false;
    }
}
