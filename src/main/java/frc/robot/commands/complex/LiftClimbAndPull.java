package frc.robot.commands.complex;

import java.util.Arrays;
import java.util.List;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.FieldPose2024;

public class LiftClimbAndPull extends Command {
	private Lift m_lift;
	private LiftSequence state = LiftSequence.VisionMoveAdjust;
	private BaseSwerveDrive m_swerveDrive;
	private Pose2d m_startClimbPose;
	private Vision m_vision;
	//private Launcher m_launcher;

	private final double DISTANCE_FROM_TAG = 0.95;
	private final double VISION_TRANSLATION_TOLERANCE = 0.01;
	private final double VISION_ROTATION_TOLERANCE = 0.01;
	private final double VISION_MOVE_SPEED = 0.15;

	private enum LiftSequence {
		MoveToArea, VisionMoveAdjust, Backup, PullDown, LiftToTrapHeight, ScoreTrap
	}

	public LiftClimbAndPull(Lift lift, BaseSwerveDrive swerveDrive, Vision vision, Launcher launcher) {
		m_lift = lift;
		m_swerveDrive = swerveDrive;
		m_vision = vision;
		// m_launcher = launcher;
		addRequirements(lift, swerveDrive, launcher);
	};

	@Override
	public void initialize() {
		// m_lift.moveToHeight(Constants.LiftConstants.StartClimbHeight);
		List<Pose2d> Stages = Arrays.asList(new Pose2d[]{
			FieldPose2024.StageSource.getCurrentAlliancePose(),
			FieldPose2024.StageAmp.getCurrentAlliancePose(),
			FieldPose2024.StageFar.getCurrentAlliancePose()});
		Pose2d closestChain = m_swerveDrive.getPose().nearest(Stages);
		double x = DISTANCE_FROM_TAG * Math.cos(closestChain.getRotation().getRadians());
		double y = DISTANCE_FROM_TAG * Math.sin(closestChain.getRotation().getRadians());
		Translation2d pose = closestChain.getTranslation().plus(new Translation2d (x, y));
		m_startClimbPose = new Pose2d(pose, closestChain.getRotation());
		m_swerveDrive.setTarget(new Pose2d(pose, closestChain.getRotation()));
	}

	@Override
	public void execute() {
		if (state == LiftSequence.MoveToArea) {
			// Swerve odometry based movement

			if(m_swerveDrive.atTarget(/* 1.5 */) && m_lift.atTargetHeight(LiftConstants.StartClimbHeight)) {
				state = LiftSequence.VisionMoveAdjust;
				m_swerveDrive.stop();
			} else {
				m_swerveDrive.moveToTarget(0.40);
				m_lift.moveToHeight(LiftConstants.StartClimbHeight);
			}

		} else if ( state == LiftSequence.VisionMoveAdjust) {
			// Vision based movement

			Pose2d visionPose = m_vision.getCamera(CameraDirection.Back).getMostRecentPose();
			Translation2d translationErrorMeters = visionPose.getTranslation().minus(m_startClimbPose.getTranslation());
			Rotation2d rotationError = visionPose.getRotation().minus(m_startClimbPose.getRotation());
			m_lift.moveToHeight(LiftConstants.StartClimbHeight);

			if(translationErrorMeters.getNorm() < VISION_TRANSLATION_TOLERANCE
					&& rotationError.getDegrees() < VISION_ROTATION_TOLERANCE
					&& m_lift.atTargetHeight(LiftConstants.StartClimbHeight))
			{
				// Once we are close enough to the target orientation and the list is raised all the way, we can move into the next state
				state = LiftSequence.Backup;

				double x = -(DISTANCE_FROM_TAG-Constants.RobotBounds.BackwardEdge) * Math.cos(m_swerveDrive.getPose().getRotation().getRadians());
				double y = -(DISTANCE_FROM_TAG-Constants.RobotBounds.BackwardEdge) * Math.sin(m_swerveDrive.getPose().getRotation().getRadians());
				Translation2d currntPose = m_swerveDrive.getPose().getTranslation();
				currntPose = currntPose.plus(new Translation2d(x, y));

				m_swerveDrive.setTarget(new Pose2d(currntPose, m_swerveDrive.getPose().getRotation()));
				
			} else { 
				var robotPose = m_swerveDrive.getPose();
				m_swerveDrive.setTarget(new Pose2d(translationErrorMeters.plus(robotPose.getTranslation()), rotationError.plus(robotPose.getRotation())));
				m_swerveDrive.moveToTarget(VISION_MOVE_SPEED);
			}

		} else if(state == LiftSequence.Backup) {
			// Lifting the launcher up the elevator

			if (m_swerveDrive.atTarget()) {
				// Once the drive is at the target, we can move the lift all the way back down.
				state = LiftSequence.PullDown;
				m_lift.moveToHeight(LiftConstants.MinHeightMeters);
				m_swerveDrive.stop();
			} else {
				// otherwise keep trying to get there
				m_swerveDrive.moveToTarget(VISION_MOVE_SPEED);
			}
		} else if(state == LiftSequence.PullDown) {
			// no work(lift do on own )- Jojo :D
		} else if(state == LiftSequence.LiftToTrapHeight) {
			// Unreachable, for possible future work
		} else if(state == LiftSequence.ScoreTrap) {
			// Unreachable, for possible future work
		}
	}

	@Override
	public boolean isFinished () {
		return m_lift.atTargetHeight(LiftConstants.MinHeightMeters) && state == LiftSequence.PullDown;
	}

	@Override
	public void end(boolean interrupted) {
		state = LiftSequence.VisionMoveAdjust;
	}
}
