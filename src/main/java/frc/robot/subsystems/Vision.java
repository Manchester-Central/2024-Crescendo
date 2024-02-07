package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision {
	private NetworkTable m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry m_botpose = m_visionTable.getEntry("botpose_wpiblue");
	private NetworkTableEntry m_pipelineID = m_visionTable.getEntry("getpipe");
	private Field2d m_field = new Field2d();

	public Rotation3d gamepeiceFinder() {
		return null;
	}

	public Pose2d getPose() {
		double[] defaults = null;
		double[] values = m_botpose.getDoubleArray(defaults);
		// If the limelight is returning bad values, return null
		if(values[0] == 0 && values[1] == 0) { 
			return null;
		} else {
		return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
		}
	}

	public void periodic() {
		if(getPose() != null) {
			m_field.setRobotPose(getPose());
		} else {
			m_field.setRobotPose(new Pose2d(0, 0, new Rotation2d(0)));
		}
		SmartDashboard.putData("vision field", m_field);
	}

	public void setPipeline() {

	}

	public int getPipeline() {
		return (int) m_pipelineID.getInteger(-1);
	}

	public double getLatencySeconds() {
		double[] defaultValues = {0, 0, 0, 0, 0, 0, 0};
		double latencyMilliseconds = m_botpose.getDoubleArray(defaultValues)[6]; // The seventh element is latency in ms
		double latencySeconds = latencyMilliseconds / 1000;
		return latencySeconds;
	}
}

