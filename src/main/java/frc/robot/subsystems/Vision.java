package frc.robot.subsystems;

import java.util.function.Supplier;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase {

	public enum CameraDirection {
     front, back

	}
	
	private LimeLightCamera m_FrontLimeLightCamera;
	private LimeLightCamera m_BackLimeLightCamera;
	private Field2d m_field = new Field2d();

	/**
	 * Creates a Vision table that attaches to a specific limelight that can be found in the Network Tables.
	 * Make sure the limelight device has a distinct static IP and network tables name.
	 * 
	 * @param tablename - String of the device name.
	 */
	public Vision(Supplier<Pose2d> simulatedPoseEstimation) {
		m_FrontLimeLightCamera = new LimeLightCamera("limelight-front" , simulatedPoseEstimation);
		m_BackLimeLightCamera = new LimeLightCamera("limelight-back" , simulatedPoseEstimation);
	}


	/**
	 * 
	 * @return The robot's current pose based on limelight april tag readings. When this function returns null,
	 * it should be ignored in calculations
	 */
	public Pose2d getPose() {
		/*if (Robot.isSimulation()) {
			return m_simPoseSupplier.get();
		}
		if(m_mode == Mode.RETROREFLECTIVE) {
			return null; // Do not run if we are currently in a non april tag pipeline
		}
		double[] defaults = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		double[] values = m_botpose.getDoubleArray(defaults);
		var xMeters = values[0];
		var yMeters = values[1];
		var avgTagDistanceMeters = values[9];
		// If the limelight is returning bad values, return null
		if((xMeters == 0 && yMeters == 0) || avgTagDistanceMeters > VisionConstants.AprilTagAverageDistanceThresholdMeters) { 
			return null;
		} else {
			return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
		} */
		return null;
	}

	public void periodic() {
		var pose = getPose();
		if(pose != null) {
			m_field.setRobotPose(pose);
		} else {
			m_field.setRobotPose(new Pose2d(0, 0, new Rotation2d(0)));
		}
		SmartDashboard.putData("vision field", m_field);


	}
}
