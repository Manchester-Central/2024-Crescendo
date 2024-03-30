package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.subsystems.vision.LimelightCamera;
import frc.robot.subsystems.vision.VisionData;
import frc.robot.subsystems.vision.LimelightCamera.LimelightVersion;

public class Vision extends SubsystemBase {

	public enum CameraDirection {
		Front, Back, Notes
	}
	
	private CameraInterface m_frontLimeLightCamera;
	private CameraInterface m_backLimeLightCamera;
	private CameraInterface m_noteTrackingCamera;

	/**
	 * Creates a Vision table that attaches to a specific limelight that can be found in the Network Tables.
	 * Make sure the limelight device has a distinct static IP and network tables name.
	 * 
	 * @param tablename - String of the device name.
	 */
	public Vision(Supplier<Pose2d> simulatedPoseEstimation, Consumer<VisionData> poseUpdator, Supplier<Double> robotSpeedSupplier) {
		m_frontLimeLightCamera = new LimelightCamera("limelight-front", LimelightVersion.LL3G, simulatedPoseEstimation, poseUpdator, robotSpeedSupplier);
		m_backLimeLightCamera = new LimelightCamera("limelight-back", LimelightVersion.LL3 , simulatedPoseEstimation, poseUpdator, robotSpeedSupplier);
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
		// TODO: Discuss if we run periodic here or in each CameraInterface
		// It comes down to how to we "lock" the pipelines, and what's the easiest system for that
	}
}
