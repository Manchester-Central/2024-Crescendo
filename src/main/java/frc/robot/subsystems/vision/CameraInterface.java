package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface CameraInterface {
	
	public enum CameraMode {
		LOCALIZATION,
		PIECE_TRACKING,
		DEMO
	};

	public void recordMeasuredData();

	public void setOffsetHandler(Supplier<Pose3d> offsetHandler);

	public Pose2d getMostRecentPose();

	public boolean hasTarget();

	public double getTargetAzimuth(boolean cameraRelative);

	public double getTargetElevation(boolean cameraRelative);

	public int getPriorityID();
	public void setPriorityID(int id);
	public void resetPriorityID();

	public CameraInterface setMode(CameraMode mode);
}
