package frc.robot.commands.simpledrive;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants2024;

public class DriveToLocation extends Command {
	private Pose2d m_targetLocation;
	private BaseSwerveDrive m_swerveDrive;
	private double m_translationToleranceMeters;
	private double m_maxPercentSpeed;

	public DriveToLocation(Pose2d location, BaseSwerveDrive swerveDrive, double translationToleranceMeters, double maxPercentSpeed) {
		m_targetLocation = location;
		m_swerveDrive = swerveDrive;
		m_translationToleranceMeters = translationToleranceMeters;
		m_maxPercentSpeed = maxPercentSpeed;
		addRequirements(swerveDrive);
	}

	public DriveToLocation(Pose2d location, BaseSwerveDrive swerveDrive) {
		this(location, swerveDrive, SwerveConstants2024.DefaultDriveToTargetTolerance_m, 1.0);
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		m_swerveDrive.driveToPositionInit();
		m_swerveDrive.resetPids();
		m_swerveDrive.setDriveTranslationTolerance(m_translationToleranceMeters);
		m_swerveDrive.setTarget(m_targetLocation.getX(), m_targetLocation.getY(), m_targetLocation.getRotation());
	}

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		m_swerveDrive.moveToTarget(m_maxPercentSpeed);
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
		m_swerveDrive.setDriveTranslationTolerance(SwerveConstants2024.DefaultDriveToTargetTolerance_m);
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
