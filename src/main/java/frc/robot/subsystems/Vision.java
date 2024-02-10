package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase {
	private NetworkTable m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry m_botpose = m_visionTable.getEntry("botpose_wpiblue");
	private NetworkTableEntry m_pipelineID = m_visionTable.getEntry("getpipe");
	private NetworkTableEntry m_tx = m_visionTable.getEntry("tx");
	
	/**
	 * Represents which mode the robot is in
	 * Localization - the current limelight pipeline uses april tags to find the current botpose
	 * Targeting - the current limelight pipeline uses CV to find targets (notes) on the field
	 */
	public enum Mode {
		BLUE_APRIL_TAGS,
		RED_APRIL_TAGS,
		RETROREFLECTIVE
  }
  
	private Field2d m_field = new Field2d();

	private Mode m_mode = Mode.RED_APRIL_TAGS; // By default we just using the limelight for localization

	/**
	 * 
	 * @return The robot's current pose based on limelight april tag readings. When this function returns null,
	 * it should be ignored in calculations
	 */
	public Pose2d getPose() {
		if(m_mode != Mode.RETROREFLECTIVE) {
			return null; // Do not run if we are currently in a non april tag pipeline
		}
		double[] defaults = {0, 0, 0, 0, 0, 0, 0};
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

	/**
	 * @return The current limelight pipeline. -1 is returned if no value is received from m_pipelineID
	 */
	public int getPipeline() {
		return (int) m_pipelineID.getInteger(-1);
	}

	public void setMode(Mode mode) {
		System.out.println(mode.ordinal());
		m_mode = mode;
		m_visionTable.getEntry("pipeline").setNumber(mode.ordinal());
	}

	public double getLatencySeconds() {
		double[] defaultValues = {0, 0, 0, 0, 0, 0, 0}; // Used if limelight values cannot be retrieved
		double latencyMilliseconds = m_botpose.getDoubleArray(defaultValues)[6]; // The seventh element is latency in ms
		double latencySeconds = latencyMilliseconds / 1000;
		return latencySeconds;
	}

	public Double noteDetection(){
		setMode(Mode.RETROREFLECTIVE);
		double temptx = m_tx.getDouble(-100);
		if(temptx < -90) return null;
		return temptx;
	}
}

