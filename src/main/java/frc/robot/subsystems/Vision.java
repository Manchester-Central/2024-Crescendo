package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.subsystems.vision.LimeLightCamera;
import frc.robot.subsystems.vision.VisionData;
import frc.robot.subsystems.vision.LimeLightCamera.LimelightVersion;


public class Vision extends SubsystemBase {

	public enum CameraDirection {
		front, back
	}
	
	private CameraInterface m_FrontLimeLightCamera;
	private CameraInterface m_BackLimeLightCamera;

	/**
	 * Creates a Vision table that attaches to a specific limelight that can be found in the Network Tables.
	 * Make sure the limelight device has a distinct static IP and network tables name.
	 * 
	 * @param tablename - String of the device name.
	 */
	public Vision(Supplier<Pose2d> simulatedPoseEstimation, Consumer<VisionData> poseUpdator, Supplier<Double> robotSpeedSupplier) {
		m_FrontLimeLightCamera = new LimeLightCamera("limelight-front" , simulatedPoseEstimation, poseUpdator, robotSpeedSupplier, LimelightVersion.LL3G);
		m_BackLimeLightCamera = new LimeLightCamera("limelight-back" , simulatedPoseEstimation, poseUpdator, robotSpeedSupplier, LimelightVersion.LL3G);
	}

	public void setOffsetHandler(Supplier<Pose3d> offsetHandler, CameraDirection direction) {
		switch (direction) {
			case front:
				m_FrontLimeLightCamera.setOffsetHandler(offsetHandler);
				break;

			case back:
				m_BackLimeLightCamera.setOffsetHandler(offsetHandler);
				break;

			default:
				break;
		}
	}

	public CameraInterface getCamera(CameraDirection direction) {
		switch (direction) {
			case front:
				return m_FrontLimeLightCamera;

			case back:
				return m_BackLimeLightCamera;

			default:
				return null;
		}
	}

	public void periodic() {
		// TODO: Discuss if we run periodic here or in each CameraInterface
		// It comes down to how to we "lock" the pipelines, and what's the easiest system for that
	}
}
