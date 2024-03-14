// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.complex;

import java.util.Optional;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.util.FieldPose2024;

public class FireIntoAmp extends Command {
	private Lift m_lift;
	private Launcher m_launcher;
	private Feeder m_feeder;
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;
	private AmpDropSequence state;
	private Pose2d m_finalPose;

	private final double VISION_TRANSLATION_TOLERANCE = 0.01;
	private final double VISION_ROTATION_TOLERANCE = 0.01;
	private final double VISION_MOVE_SPEED = 0.15;
	private final double FEEDER_EJECT_SPEED = -0.3;
	private final double FEEDER_EJECT_TIME = 1.5; // seconds
	private final double MOVE_SPEED = 0.4;
	private boolean m_hasLostNote = false;
	private Timer m_spitTimer = new Timer();

	/** Creates a new FireIntoAmp. */
	public FireIntoAmp(Lift lift, Launcher launcher, Feeder feeder, BaseSwerveDrive swerveDrive, Vision vision) {
		m_lift = lift;
		m_launcher = launcher;
		m_feeder = feeder;
		m_swerveDrive = swerveDrive;
		m_vision = vision;
		addRequirements(lift, launcher, swerveDrive, m_feeder);
		resetCommand();
	}

	public enum AmpDropSequence {
		MoveToArea, VisionMoveAdjust, FireIntoAmp
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		double x = -Constants.RobotBounds.BackwardEdge * Math.cos(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
		double y = -Constants.RobotBounds.BackwardEdge * Math.sin(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
		Translation2d pose = FieldPose2024.Amp.getCurrentAlliancePose().getTranslation().plus(new Translation2d(x, y));
		m_finalPose = new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation());
		m_swerveDrive.setTarget(m_finalPose);
		m_hasLostNote = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println(m_finalPose);
		if (state == AmpDropSequence.MoveToArea) {
			System.out.println("Move to area");
			if (m_swerveDrive.atTarget(/* 1.5 */) && m_lift.atTargetHeight(LiftConstants.AmpMeters)
				&& m_launcher.atTargetAngle(LauncherConstants.AmpAngle))
			{
				m_swerveDrive.stop();
				m_feeder.setFeederPower(-1.0);
				state = AmpDropSequence.FireIntoAmp;
			} else {
				m_lift.moveToHeight(LiftConstants.AmpMeters);
				m_launcher.setTiltAngle(LauncherConstants.AmpAngle);

				m_swerveDrive.moveToTarget(MOVE_SPEED);
			}

		} else if(state == AmpDropSequence.VisionMoveAdjust) { 
			System.out.println("Vision move adjust");

			// Calculate the error according to the Vision system
			Pose2d visionPose = m_vision.getCamera(CameraDirection.back).getMostRecentPose();
			Translation2d translationErrorMeters = visionPose.getTranslation().minus(m_finalPose.getTranslation());
			Rotation2d rotationError = visionPose.getRotation().minus(m_finalPose.getRotation());
			m_lift.moveToHeight(LiftConstants.AmpMeters);
			m_launcher.setTiltAngle(LauncherConstants.AmpAngle);

			if (translationErrorMeters.getNorm() < VISION_TRANSLATION_TOLERANCE
				&& rotationError.getDegrees() < VISION_ROTATION_TOLERANCE 
				&& m_lift.atTargetHeight(LiftConstants.AmpMeters)
				&& m_launcher.atTargetAngle(LauncherConstants.AmpAngle))
			{
				state = AmpDropSequence.FireIntoAmp;
				m_swerveDrive.stop();
				m_feeder.setFeederPower(-1.0);
			} else {
				// We need to apply the Vision error correction to the robot's current pose target
				var robotPose = m_swerveDrive.getPose();
				m_swerveDrive.setTarget(new Pose2d(translationErrorMeters.plus(robotPose.getTranslation()), rotationError.plus(robotPose.getRotation())));
				m_swerveDrive.moveToTarget(VISION_MOVE_SPEED);
			}

		} else if(state == AmpDropSequence.FireIntoAmp) {
			System.out.println("Fire to amp");
			
			if (!m_hasLostNote && !m_feeder.hasNote()) {
				m_hasLostNote = true;
				m_spitTimer.start();
			}
			// Do nothing, lets not spam the can bus
			// At this point we're timer based, and the feeder should already be ejecting as it can.
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		resetCommand();
	}

	private void resetCommand() {
		m_spitTimer.stop();
		m_spitTimer.reset();
		state = AmpDropSequence.MoveToArea;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_spitTimer.hasElapsed(0.25) && state == AmpDropSequence.FireIntoAmp) {
			m_lift.moveToHeight(LiftConstants.IntakeHeightMeters);
			return true;
		}
		return false;
	}
}

