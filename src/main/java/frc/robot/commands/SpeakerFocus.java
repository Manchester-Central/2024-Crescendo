package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive2024;

public class SpeakerFocus extends Command {
	private Pose2d m_targetLocation;
	private Translation2d m_speakerLocation;
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;

	public SpeakerFocus(Pose2d location, BaseSwerveDrive swerveDrive, Vision vision) {
		m_targetLocation = location;
		m_swerveDrive = swerveDrive;
		m_vision = vision;
		m_speakerLocation = new Translation2d(-0.0381, 5.547868);
		addRequirements(swerveDrive, vision);
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		m_swerveDrive.driveToPositionInit();
		m_swerveDrive.resetPids();
		m_swerveDrive.setTarget(m_targetLocation.getX(), m_targetLocation.getY(), m_targetLocation.getRotation());
	}
		//"No .png" -Colin 2024
	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		// m_swerveDrive.moveToTarget(0.5);
		Translation2d robotPosition = m_swerveDrive.getPose().getTranslation();
		Translation2d distance = robotPosition.minus(m_speakerLocation);
		Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan(distance.getY()/distance.getX()));
		Rotation2d currentAngle = m_swerveDrive.getPose().getRotation();
		Rotation2d angleDifference = targetAngle.minus(currentAngle);
		m_swerveDrive.moveFieldRelativeAngle(0.2, 0.2, angleDifference, 0.6);
		// "I need bed please" - Anthony 2024
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
