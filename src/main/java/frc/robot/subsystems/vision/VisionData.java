package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {
	private Pose3d m_pose;
	private double m_time;
	// TODO: Include Deviation data

	VisionData(Pose3d pose, double timestamp) {
		m_pose = pose;
		m_time = timestamp;
	}

	public Pose2d getPose2d() {
		return m_pose.toPose2d();
	}

	public Pose3d getPose3d() {
		return m_pose;
	}

	public double getTimestamp() {
		return m_time;
	}
}
