package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldPose2024;

public class SpeakerFocus extends Command {
	private BaseSwerveDrive m_swerveDrive;
	private Gamepad m_driver;

	public SpeakerFocus(BaseSwerveDrive swerveDrive, Gamepad driver) {
		m_swerveDrive = swerveDrive;
		m_driver = driver;
		addRequirements(swerveDrive);
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		m_swerveDrive.resetPids();
	}
		//"No .png" -Colin 2024
	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		// var speakerLocation = FieldPose2024.Speaker.getCurrentAlliancePose().getTranslation();
		// Translation2d robotPosition = m_swerveDrive.getPose().getTranslation();
		// Translation2d distance = robotPosition.minus(speakerLocation);
		// Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan(distance.getY()/distance.getX())).plus(Rotation2d.fromDegrees(180)); // 180 for blue - 0 for red?
		// m_swerveDrive.moveFieldRelativeAngle(m_driver.getLeftY(), -m_driver.getLeftX(), targetAngle, 0.6);
		// "I need bed please" - Anthony 2024

		var speakerLocation = FieldPose2024.Speaker.getCurrentAlliancePose().getTranslation();
		var robotPosition = m_swerveDrive.getPose().getTranslation();
		var targetAngle = speakerLocation.minus(robotPosition).getAngle();
		m_swerveDrive.moveFieldRelativeAngle(m_driver.getLeftY(), -m_driver.getLeftX(), targetAngle, 0.6);
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
		return false;
	}

}
