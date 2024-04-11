package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionData;
import frc.robot.subsystems.vision.Limelight.LimelightVersion;

public class Vision extends SubsystemBase {

	public enum CameraDirection {
		Front, Back, Notes
	}
	
	private CameraInterface m_frontLimeLightCamera;
	private CameraInterface m_backLimeLightCamera;
	private CameraInterface m_noteTrackingCamera;
	private Supplier<Rotation2d> m_robotRotationSupplier;
	private Consumer<VisionData> m_poseUpdator;

	/**
	 * Creates a Vision table that attaches to a specific limelight that can be found in the Network Tables.
	 * Make sure the limelight device has a distinct static IP and network tables name.
	 * 
	 * @param tablename - String of the device name.
	 */
	public Vision(Supplier<Pose2d> simulatedPoseEstimation, 
			Consumer<VisionData> poseUpdator, 
			Supplier<Double> robotSpeedSupplier, 
			Supplier<Double> robotRotationSpeedSupplier,
			Supplier<Rotation2d> robotRotationSupplier
			) {
		m_frontLimeLightCamera = new Limelight("limelight-front", LimelightVersion.LL3G, simulatedPoseEstimation, poseUpdator, robotSpeedSupplier, robotRotationSpeedSupplier);
		m_backLimeLightCamera = new Limelight("limelight-back", LimelightVersion.LL3 , simulatedPoseEstimation, poseUpdator, robotSpeedSupplier, robotRotationSpeedSupplier);
		m_robotRotationSupplier = robotRotationSupplier;
		m_poseUpdator = poseUpdator;
		// m_noteTrackingCamera = new LimeLightCamera("limelight-notes", LimelightVersion.LL3, simulatedPoseEstimation, poseUpdator, robotSpeedSupplier);
		// m_noteTrackingCamera.setMode(CameraMode.PIECE_TRACKING);
	}

	public void setOffsetHandler(Supplier<Pose3d> offsetHandler, CameraDirection direction) {
		switch (direction) {
			case Front:
				m_frontLimeLightCamera.setOffsetHandler(offsetHandler);
				break;

			case Back:
				m_backLimeLightCamera.setOffsetHandler(offsetHandler);
				break;

			default:
				break;
		}
	}

	public CameraInterface getCamera(CameraDirection direction) {
		switch (direction) {
			case Front:
				return m_frontLimeLightCamera;

			case Back:
				return m_backLimeLightCamera;

			case Notes:
				return m_noteTrackingCamera;

			default:
				return null;
		}
	}

	public void periodic() {
		if (!DriverStation.isEnabled()) {
			// While the robot is disabled, we pull from the megatag1 values which doesn't need any rotation information to stabilize
			double[] pose = new double[0];
			NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_wpiblue").getDoubleArray(pose);
			if (pose.length != 0) {
				//               (Yuck)
				var valid_pose = Limelight.makePoseFromArray(pose);
				if (0 < valid_pose.getX()) {
					// TODO: Clean up after the season
					var deviation = Limelight.calculateTranslationalDeviations(pose[9], pose[7]); // barf
					var trackXYZ = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{ deviation, deviation, 1 });
					var vd = new VisionData(valid_pose, Timer.getFPGATimestamp(), trackXYZ);
					if (m_poseUpdator != null) {
						m_poseUpdator.accept(vd);
					}
				};
			}
		} else {
			// TODO: Double check if this should only be enabled during teleop and autonomous.
			var rpy = new double[] {
				m_robotRotationSupplier.get().getDegrees(),
				0,
				0,
				0,
				0,
				0
			};
			// This sends the robot's current rotation to the limelight.
			// This helps megatag 2 choose a more stable position
			m_frontLimeLightCamera.updateCameraPose(rpy);
			m_backLimeLightCamera.updateCameraPose(rpy);
		}
	}
}
