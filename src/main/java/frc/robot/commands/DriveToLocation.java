package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToLocation extends Command {
	private Translation2d m_targetLocation;

	public DriveToLocation(Translation2d location) {
		m_targetLocation = location;
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		//
	}

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		//
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
		//
	}

	/**
	 * Whether the command has finished. Once a command finishes, the scheduler will call its end()
	 * method and un-schedule it.
	 *
	 * @return whether the command has finished.
	 */
	public boolean isFinished() {
		return false;
	}
}
