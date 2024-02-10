package frc.robot;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {

	public static final boolean DebugMode = true;
	public static final Alliance DefaultAlliance = Alliance.Blue;
	public static final double SimUpdateFrequency = 20.0;
	public static final boolean Use2022Robot = false;

	public static class SwerveConstants2024 {

		public static final double MaxRobotSpeed_mps = 1.0; // 5.8
		public static final double MaxRobotRotation_radps = 2.0; // 6.75 untested
		public static final PIDValue DefaultTranslationPIDValues = new PIDValue(1.0, 0.0, 0.0);
		public static final double DefaultDriveToTargetTolerance_m = 0.03; 
		public static final PIDValue DefaultRotationPIDValues = new PIDValue(0.017, 0.0001, 0.0);
		public static final Rotation2d DefaultRotationTolerance = Rotation2d.fromDegrees(3);
		public static final PIDValue DefaultModuleAnglePIDValue = new PIDValue(60.0, 12.0, 0.0);
		public static final PIDFValue DefaultModuleVelocityPIDFValues = new PIDFValue(5.0, 0.0, 0.0, 2.19);
	}	
}
