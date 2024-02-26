// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.util.FieldPose2024;

public class FireIntoAmp extends Command {
	private Lift m_lift;
	private Launcher m_launcher;
	private Feeder m_feeder;
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;
	private AmpDropSequence state;
	private Pose2d m_finalPose;
	private Optional<Double> m_timestamp = Optional.empty();

	private final double VISION_TRANSLATION_TOLERANCE = 0.01;
	private final double VISION_ROTATION_TOLERANCE = 0.01;
	private final double VISION_MOVE_SPEED = 0.15;
	private final double FEEDER_EJECT_SPEED = -0.3;
	private final double FEEDER_EJECT_TIME = 1.5; // seconds

	/** Creates a new FireIntoAmp. */
	public FireIntoAmp(Lift lift, Launcher launcher, Feeder feeder, BaseSwerveDrive swerveDrive, Vision vision) {
		m_lift = lift;
		m_launcher = launcher;
		m_feeder = feeder;
		m_swerveDrive = swerveDrive;
		m_vision = vision;
		addRequirements(lift, launcher, swerveDrive);
		resetCommand();
	}

	public enum AmpDropSequence {
		MoveToArea, VisionMoveAdjust, FireIntoAmp
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		double x = Constants.robotLengthMeters/2 * Math.cos(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
		double y = Constants.robotLengthMeters/2 * Math.sin(FieldPose2024.Amp.getCurrentAlliancePose().getRotation().getRadians());
		Translation2d pose = FieldPose2024.Amp.getCurrentAlliancePose().getTranslation().plus(new Translation2d(x, y));
		m_swerveDrive.setTarget(new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation()));
		m_finalPose = new Pose2d(pose, FieldPose2024.Amp.getCurrentAlliancePose().getRotation());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (state == AmpDropSequence.MoveToArea) {
			if (m_swerveDrive.atTarget(/* 1.5 */) && m_lift.atTargetHeight(LiftConstants.DefaultAmpMeters)
				&& m_launcher.atTargetAngle(LauncherConstants.AmpAngle))
			{
				m_swerveDrive.stop();
				state = AmpDropSequence.VisionMoveAdjust;
			} else {
				m_swerveDrive.moveToTarget(0.40);
			}

		} else if(state == AmpDropSequence.VisionMoveAdjust) { 

			// Calculate the error according to the Vision system
			Pose2d visionPose = m_vision.getPose();
			Translation2d translationErrorMeters = visionPose.getTranslation().minus(m_finalPose.getTranslation());
			Rotation2d rotationError = visionPose.getRotation().minus(m_finalPose.getRotation());
			m_lift.moveToHeight(LiftConstants.AmpHeight);
			m_launcher.setTiltAngle(LauncherConstants.AmpAngle);

			if (translationErrorMeters.getNorm() < VISION_TRANSLATION_TOLERANCE
				&& rotationError.getDegrees() < VISION_ROTATION_TOLERANCE 
				&& m_lift.atTargetHeight(LiftConstants.AmpHeight)
				&& m_launcher.atTargetAngle(LauncherConstants.AmpAngle))
			{
				state = AmpDropSequence.FireIntoAmp;
				m_swerveDrive.stop();
				m_timestamp = Optional.of(Timer.getFPGATimestamp());
				m_feeder.setFeederPower(FEEDER_EJECT_SPEED);
			} else {
				// We need to apply the Vision error correction to the robot's current pose target
				var robotPose = m_swerveDrive.getPose();
				m_swerveDrive.setTarget(new Pose2d(translationErrorMeters.plus(robotPose.getTranslation()), rotationError.plus(robotPose.getRotation())));
				m_swerveDrive.moveToTarget(VISION_MOVE_SPEED);
			}

		} else if(state == AmpDropSequence.FireIntoAmp) {
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
		state = AmpDropSequence.VisionMoveAdjust;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_timestamp.isEmpty()) {
			return false;
		}

		var time_diff = (Timer.getFPGATimestamp() - m_timestamp.get()) / 1000;
		if (state == AmpDropSequence.FireIntoAmp && FEEDER_EJECT_TIME < time_diff) {
			return true;
		} else {
			return false;
		}
	}
}
