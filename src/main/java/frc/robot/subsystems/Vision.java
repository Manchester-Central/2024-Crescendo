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
import frc.robot.Constants.VisionConstants;


public class Vision {
	private NetworkTable m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry m_botpose = m_visionTable.getEntry("botpose_wpiblue");
	private NetworkTableEntry m_pipelineID = m_visionTable.getEntry("getpipe");
	private Field2d m_field;
	
	/**
	 * Represents which mode the robot is in
	 * Localization - the current limelight pipeline uses april tags to find the current botpose
	 * Targeting - the current limelight pipeline uses CV to find targets (notes) on the field
	 */
	public enum Mode {
		APRIL_TAGS, RETROREFLECTIVE
	}

	private Mode m_mode = Mode.APRIL_TAGS; // By default we just using the limelight for localization

	/**
	 * 
	 * @return The robot's current pose based on limelight april tag readings. When this function returns null,
	 * it should be ignored in calculations
	 */
	public Pose2d getPose() {
		if(m_mode != Mode.APRIL_TAGS) {
			return null; // Do not run if we are currently in a non april tag pipeline
		}
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
		m_field.setRobotPose(getPose());
		SmartDashboard.putData("vision field", m_field);
	}

	public void setPipeline(int pipeline) {
		m_pipelineID.setInteger(pipeline);
		for(int aprilTagPipeline : VisionConstants.AprilTagPipelines) {
			if(pipeline == aprilTagPipeline) {
				setMode(Mode.APRIL_TAGS);
			}
		}
	}

	/**
	 * @return The current limelight pipeline. -1 is returned if no value is received from m_pipelineID
	 */
	public int getPipeline() {
		return (int) m_pipelineID.getInteger(-1);
	}

	public void setMode(Mode mode) {
		m_mode = mode;
	}

	public double getLatencySeconds() {
		double[] defaultValues = {0, 0, 0, 0, 0, 0, 0}; // Used if limelight values cannot be retrieved
		double latencyMilliseconds = m_botpose.getDoubleArray(defaultValues)[6]; // The seventh element is latency in ms
		double latencySeconds = latencyMilliseconds / 1000;
		return latencySeconds;
	}
}

