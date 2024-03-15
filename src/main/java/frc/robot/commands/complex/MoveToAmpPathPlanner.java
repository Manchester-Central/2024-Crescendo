package frc.robot.commands.complex;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.jsontype.impl.AsDeductionTypeSerializer;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

public class MoveToAmpPathPlanner extends Command {
	private SwerveDrive m_swerve;
	private PathPlannerPath m_path;

	public MoveToAmpPathPlanner(SwerveDrive swerve) {
		m_swerve = swerve;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		var amp = FieldPose2024.Amp.getCurrentAlliancePose();

		ArrayList<Translation2d> bezierPoints = new ArrayList<>();
		Translation2d destination = amp.getTranslation().minus(new Translation2d(Constants.RobotBounds.BackwardEdge, amp.getRotation()));
		Translation2d destination_control_point = destination.plus(new Translation2d(3, amp.getRotation()));

		Translation2d current_pose = m_swerve.getPose().getTranslation();
		Translation2d current_pose_control_point = current_pose;

		bezierPoints.add(current_pose);
		bezierPoints.add(current_pose_control_point);
		bezierPoints.add(destination);
		bezierPoints.add(destination_control_point);

		PathConstraints pc = new PathConstraints(SwerveConstants2024.MaxRobotSpeed_mps,
												SwerveConstants2024.MaxRobotAcceleration_mps2,
												SwerveConstants2024.MaxRobotRotation_radps,
												SwerveConstants2024.MaxRobotRotationAccel_radps2);
		GoalEndState gs = new GoalEndState(0, amp.getRotation());
		
		m_path = new PathPlannerPath(bezierPoints, pc, null);
	}

	@Override
	public void execute() {
		//
	}

	@Override
	public void end(boolean isInterrupted) {
		//
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
