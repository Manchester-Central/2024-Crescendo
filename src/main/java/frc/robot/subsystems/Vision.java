package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.*;
	
public class Vision extends SubsystemBase {
	private NetworkTable m_visionTable;
	private NetworkTableEntry m_botpose;
	private NetworkTableEntry m_pipelineID;
	private NetworkTableEntry m_json;

	// An overloaded variable, this stores targetting info from any pipeline in a double value for the focus point's azimuth.
	private NetworkTableEntry m_tx;
	private NetworkTableEntry m_ty;
	
	/**
	 * Represents which mode the robot is in.
	 * 
	 * <p>BLUE_APRIL_TAGS - The pipeline corresponding to the tags on the BLUE side of the field.
	 * <p>RED_APRIL_TAGS - The pipeline corresponding to the tags on the RED side of the field.
	 * <p>RETROREFLECTIVE - The pipeline used for finding notes on the field.
	 * This is typically for intake cameras, which may not be the forward camera.
	 */
	public enum Mode {
		BLUE_APRIL_TAGS,
		RED_APRIL_TAGS,
		RETROREFLECTIVE
	}

	private Field2d m_field = new Field2d();

	private Mode m_mode = Mode.BLUE_APRIL_TAGS; // By default we just using the limelight for localization

	/**
	 * Creates a Vision table that attaches to a specific limelight that can be found in the Network Tables.
	 * Make sure the limelight device has a distinct static IP and network tables name.
	 * 
	 * @param tablename - String of the device name.
	 */
	public Vision(String tablename) {
		m_visionTable = NetworkTableInstance.getDefault().getTable(tablename);
		m_botpose = m_visionTable.getEntry("botpose_wpiblue");
		m_pipelineID = m_visionTable.getEntry("getpipe");
		m_tx = m_visionTable.getEntry("tx");
		m_ty = m_visionTable.getEntry("ty");
		m_json = m_visionTable.getEntry("json");
	}

	/**
	 * Overloaded for convenience to be able to set the initial mode/state.
	 * 
	 * @param tablename - String of the device name.
	 * @param initialState - Whichever state (ie, side of the field) the robot is initializing on.
	 */
	public Vision(String tablename, Mode initialState) {
		this(tablename);
		m_mode = initialState;
	} 

	/**
	 * 
	 * @return The robot's current pose based on limelight april tag readings. When this function returns null,
	 * it should be ignored in calculations
	 */
	public Pose2d getPose() {
		if(m_mode == Mode.RETROREFLECTIVE) {
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
	@Override
	public void periodic() {
		if(getPose() != null) {
			m_field.setRobotPose(getPose());
		} else {
			m_field.setRobotPose(new Pose2d(0, 0, new Rotation2d(0)));
		}
		SmartDashboard.putData("vision field", m_field);
		SmartDashboard.putString("readJSON", m_json.getString("no value received"));
	}

	/**
	 * Pulls the current limelight pipeline number from network tables.
	 * 
	 * @return -1 is returned if no value is received from m_pipelineID
	 */
	public long getPipeline() {
		return m_pipelineID.getInteger(-1);
	}

	/**
	 * Changes the internal state machine, and updates the limelight to use the specified 
	 * 
	 * @param mode - The Mode to switch to (BLUE_APRIL_TAGS, RED_APRIL_TAGS, RETROREFLECTIVE)
	 * @return - Returns itself for chaining
	 */
	public Vision setMode(Mode mode) {
		m_mode = mode;
		m_visionTable.getEntry("pipeline").setNumber(mode.ordinal());

		return this;
	}

	public double getLatencySeconds() {
		double[] defaultValues = {0, 0, 0, 0, 0, 0, 0}; // Used if limelight values cannot be retrieved
		double latencyMilliseconds = m_botpose.getDoubleArray(defaultValues)[6]; // The seventh element is latency in ms
		double latencySeconds = latencyMilliseconds / 1000;
		return latencySeconds;
	}

	/**
	 * On the first attempt to detect a note from a different pipeline, it will have to tell the limelight to swap back.
	 * In an effort to reduce bad values leaking into the system, we reject any values that might be there since AprilTags
	 * still return valid tx and ty values.
	 * 
	 * <p>NOTE: (pun intended) that latency in the system could still introduce incorrect values that are ordinarily valid.
	 * This needs further research, but may not be a problem in practice. Or maybe we just remove values above the horizon?
	 * 
	 * @return - The azimuth angle in degrees where +theta is to the right (typically opposite from robot drive)
	 */
	public Double getNoteAzimuth() {
		if (m_mode != Mode.RETROREFLECTIVE) {
			setMode(Mode.RETROREFLECTIVE);
			return null;
		}
		Double temptx = m_tx.getDouble(-100);
		if(temptx < -90) return null;
		return temptx;
	}
}
