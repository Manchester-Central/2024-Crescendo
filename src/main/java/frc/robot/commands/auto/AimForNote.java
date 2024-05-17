package frc.robot.commands.auto;

import java.nio.Buffer;
import java.util.LinkedList;
import java.util.Queue;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.subsystems.vision.CameraInterface.CameraMode;

public class AimForNote extends Command {
	private final int minDataPoints = 5; // rough guess
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;
	private Intake m_intake;
	private CameraInterface m_camera;
	private final double m_speed = -0.5; // needs to be in range [-1.0, 1.0] negative values are backwards
	private Launcher m_launcher;
	private Feeder m_feeder;
	private double m_startime;
	private final double m_minDurationSeconds = 1.0;
	private LinkedList<Double> m_queue = new LinkedList<>(); // fix this

	public AimForNote(BaseSwerveDrive swerveDrive, Vision vision, Intake intake, Launcher launcher, Feeder feeder){
		m_swerveDrive = swerveDrive;
		m_intake = intake;
		m_vision = vision;
		m_camera = m_vision.getCamera(CameraDirection.Back);
		m_launcher = launcher;
		m_feeder = feeder;
		addRequirements(swerveDrive, intake, vision, launcher, feeder);
	}

	public void initialize() {
		m_vision.getCamera(CameraDirection.Back).setMode(CameraMode.PIECE_TRACKING);
		m_startime = Timer.getFPGATimestamp();
		m_intake.setIntakePower(1);
	}

	/** The main body of a command. Called repeatedly while the command is scheduled. */
	public void execute() {
		m_feeder.grabAndHoldPiece(0.35);
		m_launcher.setTiltAngle(Constants.LauncherConstants.VisionIntakeAngle);

		if (!m_camera.isCorrectPipeline()) {
			// lets protect ourselves from targetting an april tag and moving to it
			return;
		}


		Double tx = m_camera.getTargetAzimuth(true);
		Double ty = m_camera.getTargetElevation(true);
		m_queue.add(ty);
		if (m_queue.size() >= minDataPoints) {
			m_queue.getLast();
		}


		Rotation2d noteAngle = m_swerveDrive.getOdometryRotation().rotateBy( Rotation2d.fromDegrees(-tx) );
		var speedModifier = MathUtil.clamp(ty/50, 0.1, 1.0);
		Translation2d speed = new Translation2d(m_speed * speedModifier, noteAngle);
		m_swerveDrive.moveFieldRelativeAngle(speed.getX(), speed.getY(), noteAngle, 0.75);
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
		m_camera.setMode(CameraMode.LOCALIZATION);
		m_intake.setIntakePower(0.0);
	}

	/**
	 * Whether the command has finished. Once a command finishes, the scheduler will call its end()
	 * method and un-schedule it.
	 *
	 * @return whether the command has finished.
	 */
	public boolean isFinished() {
		if (m_feeder.hasNote()) {
			return true;
		}
		if (m_camera.hasTarget()) {
			return false;
		}
		if (Timer.getFPGATimestamp()-m_startime < m_minDurationSeconds) {
			return false;
		}
		return true;
	}
}
