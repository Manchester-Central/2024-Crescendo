package frc.robot.commands;

import com.chaos131.swerve.BaseSwerveDrive;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToLocation extends Command {
	private Translation2d m_targetLocation;
	private BaseSwerveDrive m_swerveDrive;

	public DriveToLocation(Translation2d location, BaseSwerveDrive swerveDrive) {
		m_targetLocation = location;
		m_swerveDrive = swerveDrive;
		addRequirements(swerveDrive);
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		m_swerveDrive.driveToPositionInit();
		m_swerveDrive.resetPids();
		m_swerveDrive.setTarget(m_targetLocation.getX(), m_targetLocation.getY(), m_swerveDrive.getOdometryRotation());
	}

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		m_swerveDrive.moveToTarget(0.5);
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
		m_swerveDrive.resetPids();
		m_swerveDrive.stop();
	}

	/**
	 * Whether the command has finished. Once a command finishes, the scheduler will call its end()
	 * method and un-schedule it.
	 *
	 * @return whether the command has finished.
	 */
	public boolean isFinished() {
		return m_swerveDrive.atTarget();
	}
}
