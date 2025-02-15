package frc.robot.subsystems.vision;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class Limelight implements CameraInterface {
	private String m_name;
	private Mode m_mode = Mode.BLUE_APRIL_TAGS; // By default we just using the limelight for localization
	private LimelightVersion m_limeLightVersion;

	private Supplier<Pose2d> m_simPoseSupplier; // supplies data when in simulation
	private Consumer<VisionData> m_poseUpdator; // sends the data back to the swerve pose estimator
	private Consumer<Pose3d> m_demoTargetUpdater; // sends the target pose to back to Robot Container
	private Optional<VisionData> m_mostRecentData; // caches the most recent data, including no-datas
	private Supplier<Pose3d> m_offset;
	private Supplier<Double> m_robotSpeedSupplier;
	private Supplier<Double> m_robotRotationSpeedSupplier;

	private NetworkTable m_visionTable;
	private NetworkTableEntry m_botPose;
	private NetworkTableEntry m_demoPose;
	private NetworkTableEntry m_pipelineID;

	// An overloaded variable, this stores targetting info from any pipeline in a double value for the focus point's azimuth.
	private NetworkTableEntry m_tx;
	private NetworkTableEntry m_ty;
	private NetworkTableEntry m_tv;
	private NetworkTableEntry m_priorityid;

	@AutoLogOutput(key = "Limelight/{m_name}/Demo/RawCameraPose")
	private Pose3d m_demoRawCameraPose = new Pose3d();
	@AutoLogOutput(key = "Limelight/{m_name}/Demo/AprilTagPose")
	private Pose3d m_demoAprilTagPose = new Pose3d();
	@AutoLogOutput(key = "Limelight/{m_name}/Demo/TargetPose")
	private Pose3d m_demoTargetPose = new Pose3d();

	public enum LimelightVersion {
		LL2,
		LL3,
		LL3G
	}


	/**
	 * Represents which mode the robot is in.
	 * 
	 * <p>BLUE_APRIL_TAGS - The pipeline corresponding to the tags on the BLUE side of the field.
	 * <p>RED_APRIL_TAGS - The pipeline corresponding to the tags on the RED side of the field.
	 * <p>RETROREFLECTIVE - The pipeline used for finding notes on the field.
	 * This is typically for intake cameras, which may not be the forward camera.
	 */
	public enum Mode {
		BLUE_APRIL_TAGS(0),
		RED_APRIL_TAGS(1),
		RETROREFLECTIVE(2),
		BLUE_SPEAKER(3),
		RED_SPEAKER(4),
		DEMO(5);

		public final Integer pipelineId;

		private Mode(Integer pipelineId) {
			this.pipelineId = pipelineId;
		}
	}

	private final double EPSILON = 1e-8;

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

	public Limelight(String name, LimelightVersion limelightVersion, Supplier<Pose2d> poseSupplier, Consumer<VisionData> poseConsumer, Consumer<Pose3d> demoTargetUpdater, Supplier<Double> robotSpeedSupplier, Supplier<Double> robotRotationSpeedSupplier) {
		m_name = name;
		m_visionTable = NetworkTableInstance.getDefault().getTable(m_name);
		m_demoPose = m_visionTable.getEntry("targetpose_robotspace");
		m_botPose = m_visionTable.getEntry("botpose_wpiblue");
		m_pipelineID = m_visionTable.getEntry("getpipe");
		m_tx = m_visionTable.getEntry("tx");
		m_ty = m_visionTable.getEntry("ty");
		m_tv = m_visionTable.getEntry("tv");
		m_priorityid = m_visionTable.getEntry("priorityid");
		m_simPoseSupplier = poseSupplier;
		m_poseUpdator = poseConsumer;
		m_demoTargetUpdater = demoTargetUpdater;
		m_robotSpeedSupplier = robotSpeedSupplier;
		m_robotRotationSpeedSupplier = robotRotationSpeedSupplier;
		m_limeLightVersion = limelightVersion;

		m_visionTable.addListener("botpose_wpiblue", EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
										(NetworkTable table, String key, NetworkTableEvent event) -> {
											recordMeasuredData();
										});

		m_visionTable.addListener("targetpose_cameraspace", EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
										(NetworkTable table, String key, NetworkTableEvent event) -> {
											recordDemoPoseData();
										});
										

		if (Robot.isSimulation()) {
			// Fake demo target data in sim
			var simVisionTask = new Thread(() -> {
				while(true) {
					if (DriverStation.isDisabled()) {
						m_demoPose.setDoubleArray(new double[]{1, 1, 1, 45, 0, 0});
						recordDemoPoseData();
					}
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			});
			simVisionTask.start();
		}

		m_mostRecentData = Optional.empty();
	}

	private double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation) {
		// TODO Actually calculate confidence
		// if (VisionConstants.PoseDeviationThreshold < deviation) {
		// 	return 0;
		// }
		var rotationSpeed = Math.abs(m_robotRotationSpeedSupplier.get());
		var isMovingTooFast = m_limeLightVersion == LimelightVersion.LL3G ? false : VisionConstants.RobotSpeedThresholdMPS < m_robotSpeedSupplier.get();
		var isRotatingTooFast = m_limeLightVersion == LimelightVersion.LL3G ? rotationSpeed > 0.8 : rotationSpeed > 0.2; // TODO: change values and make constants
		var isTooFar =  m_limeLightVersion == LimelightVersion.LL3G ? distance > 4: VisionConstants.AprilTagAverageDistanceThresholdMeters < distance;
		if (isTooFar || isMovingTooFast || isRotatingTooFast) {
			return 0;
		}
		return 1.0;
	}

	private double calculateTranslationalDeviations(double distance, double tagCount) {
		var stddev = Math.pow(distance*Constants.VisionConstants.LL3G.DistanceScalar, Constants.VisionConstants.LL3G.ErrorExponent);
		// commented out for now until we know what's what
		// stddev /= tagCount * VisionConstants.L3G.TagCountErrorScalar;
		// stddev *= (1 + m_robotSpeedSupplier.get() * VisionConstants.LL3G.RobotSpeedErrorScalar);

		return VisionConstants.LL3G.TotalDeviationMultiplier * stddev + VisionConstants.LL3G.MinimumError;
	}

	private Pose3d convertFieldPoseNTEntryToPose3D(double[] limelightPoseData) {
		var poseRotation = new Rotation3d(	limelightPoseData[idxRoll] * Math.PI / 180, 
											limelightPoseData[idxPitch] * Math.PI / 180,
											limelightPoseData[idxYaw]  * Math.PI / 180);

		return new Pose3d(limelightPoseData[idxX], limelightPoseData[idxY], limelightPoseData[idxZ], poseRotation);
	}

	private Pose3d convertCameraPoseNTEntryToPose3D(double[] limelightPoseData) {
		var poseRotation = new Rotation3d(	-1 * limelightPoseData[idxYaw] * Math.PI / 180, 
											limelightPoseData[idxRoll] * Math.PI / 180,
											-1 * (limelightPoseData[idxPitch]  * Math.PI / 180) + Math.PI
											);

		return new Pose3d(limelightPoseData[idxZ], -limelightPoseData[idxX], -limelightPoseData[idxY], poseRotation);
	}

	private void recordDemoPoseData() {
		var data = m_demoPose.getValue().getDoubleArray(); // TODO: we probably want robot space target to simplify math here?
		if (data == null || data.length == 0) {
			return;
		}
		var robotPose = m_simPoseSupplier.get();
		m_demoRawCameraPose = convertCameraPoseNTEntryToPose3D(data);
		m_demoAprilTagPose = new Pose3d(robotPose).plus(new Transform3d(m_demoRawCameraPose.getTranslation(), m_demoRawCameraPose.getRotation()));
		//0.476 distance to middle of the net in meters
		m_demoTargetPose = m_demoAprilTagPose.plus(new Transform3d(new Translation3d(0, 0, 0.316), new Rotation3d()));
		m_demoTargetUpdater.accept(m_demoTargetPose);
	}

	/**
	 * 
	 */
	@Override
	public void recordMeasuredData() {
		var data = m_botPose.getValue().getDoubleArray();
		double timestampSeconds = Timer.getFPGATimestamp() - data[idxLatency] / 1000;
		if (data == null || data[idxX] < EPSILON) {
			m_mostRecentData = Optional.empty();
			return;
		}

		var visionPose = convertFieldPoseNTEntryToPose3D(data);

		if (m_offset != null) {
			var cameraOffset = m_offset.get();
			cameraOffset = cameraOffset.rotateBy(new Rotation3d(0, 0, data[idxYaw] * Math.PI / 180));
			visionPose = new Pose3d(
				new Translation3d(visionPose.getX() - cameraOffset.getX(),
								visionPose.getY() - cameraOffset.getY(),
								visionPose.getZ() - cameraOffset.getZ()),
				new Rotation3d(0,//poseRotation.getX() - cameraOffset.getRotation().getX(),
								0,//poseRotation.getY() - cameraOffset.getRotation().getY(),
								visionPose.getRotation().getZ())//- cameraOffset.getRotation().getZ())
			);
		}

		var deviation = calculateTranslationalDeviations(data[idxTagDistance], data[idxTagCount]);
		var trackXYZ = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{ deviation, deviation, 1 });

		var conf = calculateConfidence(visionPose, (int)data[idxTagCount], data[idxTagDistance], deviation);
		if (conf < VisionConstants.ConfidenceRequirement) {
			m_mostRecentData = Optional.empty();
			return;
		}

		m_mostRecentData = Optional.of(new VisionData(visionPose, timestampSeconds, trackXYZ));
		if (m_poseUpdator != null && Constants.VisionConstants.UseVisionForOdometry) {
			m_poseUpdator.accept(m_mostRecentData.get());
		}
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
	public CameraInterface setMode(CameraMode mode) {
		switch (mode) {
			case LOCALIZATION:
				m_mode = Mode.BLUE_APRIL_TAGS;
				break;

			case PIECE_TRACKING:
				m_mode = Mode.RETROREFLECTIVE;
				break;

			case DEMO:
				m_mode = Mode.DEMO;
				break;
		
			default:
				return this;
		}
		m_visionTable.getEntry("pipeline").setNumber(m_mode.pipelineId);

		return this;
	}

	private boolean isCorrectPipeline() {
		return getPipeline() == m_mode.pipelineId;
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
	public void setToDefaultMode(Pose2d robotPose){
		setMode(CameraMode.LOCALIZATION);
	}

	@Override
	public Pose2d getMostRecentPose() {
		if(Robot.isSimulation()) {
			return m_simPoseSupplier.get();
		}
		if(m_mostRecentData.isPresent()) {
			return m_mostRecentData.get().getPose2d();
		}
		return null;
	}

	@Override
	public boolean hasTarget() {
		if(Robot.isSimulation()) {
			return true;
		}
		return m_tv.getInteger(-1) == 1;
	}

	@Override
	public void setOffsetHandler(Supplier<Pose3d> offsetHandler) {
		m_offset = offsetHandler;
	}

	@Override
	public double getTargetAzimuth(boolean cameraRelative) {
		if (Robot.isSimulation()) {
			return 0.0;
		}
		Double temptx = m_tx.getDouble(-100);
		if(temptx < -90) return -90;
		return temptx;
	}

	@Override
	public double getTargetElevation(boolean cameraRelative) {
		if (Robot.isSimulation()) {
			return 0.0;
		}
		Double tempty = m_ty.getDouble(-100);
		if(tempty < -90) return -90;
		return tempty;
	}

	/**
	 * Returns the ApriltTag ID of the current tx/ty target
	 */
	@Override
	public int getPriorityID() {
		return (int) m_priorityid.getInteger(-1);
	}

	/**
	 * ID must be the number of the tag you want to track. -1 to reset it.
	 */
	@Override
	public void setPriorityID(int id) {
		m_priorityid.setNumber(id);
	}

	public void resetPriorityID() {
		m_priorityid.setNumber(-1);
	}
}
