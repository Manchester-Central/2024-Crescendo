package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.CameraDirection;

public class LimeLightCamera {
    private String m_name;
    private Mode m_mode = Mode.BLUE_APRIL_TAGS; // By default we just using the limelight for localization
    private Supplier<Pose2d> m_simPoseSupplier;
    private NetworkTable m_visionTable;
	private NetworkTableEntry m_botpose;
	private NetworkTableEntry m_pipelineID;

	// An overloaded variable, this stores targetting info from any pipeline in a double value for the focus point's azimuth.
	private NetworkTableEntry m_tx;
	private NetworkTableEntry m_ty;
	private NetworkTableEntry m_tv;

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
		RETROREFLECTIVE,
		BLUE_SPEAKER,
		RED_SPEAKER
	}

    private final double EPSILON = 1e-8;
	private final double CONFIDENCE_REQUIREMENT = 0.4;
	private final double DISTANCE_CUTOFF_METERS = 4;

	private final int idxX = 0;
	private final int idxY = 1;
	private final int idxZ = 2;
	private final int idxRoll = 3;
	private final int idxPitch = 4;
	private final int idxYaw = 5;
	private final int idxLatency = 6;
	private final int idxTagCount = 7;
	private final int idxTagSpan = 8;
	private final int idxTagDistance = 9;
	private final int idxTagArea = 10;

    public LimeLightCamera(String name, Supplier<Pose2d> poseSupplier) {
        m_name = name;
        m_visionTable = NetworkTableInstance.getDefault().getTable(name);
		m_botpose = m_visionTable.getEntry("botpose_wpiblue");
		m_pipelineID = m_visionTable.getEntry("getpipe");
		m_tx = m_visionTable.getEntry("tx");
		m_ty = m_visionTable.getEntry("ty");
		m_tv = m_visionTable.getEntry("tv");
        m_simPoseSupplier = poseSupplier;

        m_visionTable.addListener("botpose_wpiblue", EnumSet.of(NetworkTableEvent.Kind.kValueAll),
										 (NetworkTable table, String key, NetworkTableEvent event) -> {
                                        // TODO: finish 
                                        // recordMeasuredData(table.getValue(key).getDoubleArray(), CameraDirection.front);
										});
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
	public LimeLightCamera setMode(Mode mode) {
		m_mode = mode;
		m_visionTable.getEntry("pipeline").setNumber(mode.ordinal());

		return this;
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

    public void setToDefaultMode(Pose2d robotPose){
		if(robotPose.getX() < VisionConstants.XMetersMidPoint){
			setMode(Mode.BLUE_APRIL_TAGS);
		}else{
			setMode(Mode.RED_APRIL_TAGS);
		}
	}
}
