// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.complex;

import java.util.ArrayList;
import java.util.Optional;

import com.chaos131.swerve.BaseSwerveDrive;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

public class FireIntoAmp extends Command {
	private Lift m_lift;
	private Launcher m_launcher;
	private Feeder m_feeder;
	private BaseSwerveDrive m_swerveDrive;
	private Vision m_vision;
	private AmpDropSequence state;
	private Pose2d m_finalPose;

	private PathPlannerPath m_path;
	private Command m_pathCommand;

	private final double VISION_TRANSLATION_TOLERANCE = 0.01;
	private final double VISION_ROTATION_TOLERANCE = 0.01;
	private final double VISION_MOVE_SPEED = 0.15;
	private final double FEEDER_EJECT_SPEED = -1.0;
	private final double FEEDER_EJECT_TIME = 1.5; // seconds
	private final double MOVE_SPEED = 0.4;
	private final double fiaAmpHeight = 0.64;
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
		var amp = FieldPose2024.Amp.getCurrentAlliancePose();

		ArrayList<Translation2d> bezierPoints = new ArrayList<>();
		ArrayList<Rotation2d> rotationPoints = new ArrayList<>();
		Translation2d destination = amp.getTranslation().minus(new Translation2d(Constants.RobotBounds.BackwardEdge*1.2, amp.getRotation()));
		Translation2d destination_control_point = destination.plus(new Translation2d(2, amp.getRotation()));

		Translation2d current_pose = m_swerveDrive.getPose().getTranslation();
		Translation2d current_pose_control_point = current_pose;

		bezierPoints.add(current_pose);
		bezierPoints.add(current_pose_control_point);
		bezierPoints.add(destination_control_point);
		bezierPoints.add(destination);

		PathConstraints pc = new PathConstraints(SwerveConstants2024.MaxRobotSpeed_mps,
												3.0,
												SwerveConstants2024.MaxRobotRotation_radps,
												3*Math.PI);
		GoalEndState gs = new GoalEndState(0, amp.getRotation(), true);
		
		m_path = new PathPlannerPath(bezierPoints, pc, gs);
		m_path.preventFlipping = true;
		m_pathCommand = AutoBuilder.followPath(m_path);

		m_pathCommand.initialize();
		m_hasLostNote = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (state == AmpDropSequence.MoveToArea) {
			if (m_pathCommand.isFinished() && m_lift.atTargetHeight(fiaAmpHeight)
				&& m_launcher.atTargetAngle(LauncherConstants.AmpAngle)) {
				m_feeder.setFeederPower(FEEDER_EJECT_SPEED);
				state = AmpDropSequence.FireIntoAmp;
			} else {
				m_pathCommand.execute();
				m_lift.moveToHeight(fiaAmpHeight);
				m_launcher.setTiltAngle(LauncherConstants.AmpAngle);
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

