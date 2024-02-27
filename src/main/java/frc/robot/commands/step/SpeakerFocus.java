package frc.robot.commands.step;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.util.AngleUtil;
import frc.robot.util.FieldPose2024;

public class SpeakerFocus extends Command {
	private BaseSwerveDrive m_swerveDrive;
	private Gamepad m_driver;
	private Vision m_vision;

	public SpeakerFocus(BaseSwerveDrive swerveDrive, Gamepad driver, Vision vision) {
		m_swerveDrive = swerveDrive;
		m_driver = driver;
		m_vision = vision;
		addRequirements(swerveDrive);
	}

	/** Runs when the command is first run, before execute. It is only run once. */
	public void initialize() {
		m_swerveDrive.resetPids();
		var visionMode = DriverStation.getAlliance().get() == Alliance.Blue ? Vision.Mode.BLUE_SPEAKER : Vision.Mode.RED_SPEAKER;
		m_vision.setMode(visionMode);
	}

		//"No .png" -Colin 2024
	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		// Translation2d robotPosition = m_swerveDrive.getPose().getTranslation();
		// Translation2d cameraPosition = m_vision.getPose().getTranslation();
		// if (cameraPosition == null) {
		// 	m_swerveDrive.moveFieldRelative(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), -m_driver.getSlewRightX());
		// }
		// Translation2d speakerLocation = FieldPose2024.Speaker.getCurrentAlliancePose().getTranslation();
		// Translation2d distance = speakerLocation.minus(robotPosition);
		// Rotation2d targetAngle = distance.getAngle();
		// m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), targetAngle, 0.6);
		if (m_vision.hasTarget()) {
			var currentPose = m_swerveDrive.getPose();
			var currentRotation = currentPose.getRotation();
			var rotation = AngleUtil.GetEstimatedAngleToGoal(m_vision, currentPose, currentRotation);
			m_swerveDrive.moveFieldRelativeAngle(m_driver.getSlewLeftY(), -m_driver.getSlewLeftX(), rotation, 1.0);
		}
		// // "I need bed please" - Anthony 2024
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
