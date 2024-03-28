package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionData {
	private Pose3d m_pose;
	private double m_time;
	private Matrix <N3,N1> m_devation;

	VisionData(Pose3d pose, double timestamp, Matrix <N3,N1>devation) {
		m_pose = pose;
		m_time = timestamp;
		m_devation = devation;
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

	public Matrix <N3,N1> getDevation(){
		return m_devation; 
	}
}
