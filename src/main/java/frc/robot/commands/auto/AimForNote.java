package frc.robot.commands.auto;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Translation2dProto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.vision.CameraInterface.CameraMode;

public class AimForNote extends Command {
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;
	private final double m_speed = -0.1; // needs to be in range [-1.0, 1.0] negative values are backwards

	public AimForNote(BaseSwerveDrive swerveDrive, Vision vision){
		m_swerveDrive = swerveDrive;
		m_vision = vision;
		addRequirements(swerveDrive, vision);
	}

	public void initialize() {
		m_vision.getCamera(CameraDirection.Back).setMode(CameraMode.PIECE_TRACKING);
		
	}

	

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		Double tx = m_vision.getCamera(CameraDirection.Back).getTargetAzimuth(true);

		
		if(!m_vision.getCamera(CameraDirection.Back).hasTarget()) return;

		Rotation2d noteAngle = m_swerveDrive.getOdometryRotation().rotateBy( Rotation2d.fromDegrees(-tx) );
		Translation2d speed = new Translation2d(m_speed, noteAngle);
		// TODO: Drive into the note until it is intaken (intook?)
		m_swerveDrive.moveFieldRelativeAngle(speed.getX(), speed.getY(), noteAngle, 0.75);
	}
//
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
		m_vision.getCamera(CameraDirection.Back).setMode(CameraMode.LOCALIZATION);
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
