package frc.robot;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
	public static final boolean Use2022Robot = false;

	public static final boolean DebugMode = true;
	public static final Alliance DefaultAlliance = Alliance.Blue;
	public static final double SimUpdateFrequency = 20.0;

	public static final double robotWidthMeters = 0.990;
	public static final double robotHeightMeters = 0.876;
	public static final double robotLengthMeters = 0.9779;

	public static class DebugConstants {
		public static final boolean DriveDebugEnable = false;
		public static final boolean IntakeDebugEnable = false;
		public static final boolean FeederDebugEnable = false;
		public static final boolean LiftDebugEnable = false;
		public static final boolean LauncherDebugEnable = false;
	}

	public static class ControllerConstants {
		public static final int DriverPort = 0;
		public static final int OperatorPort = 1;
		public static final int SimKeyboardPort = 2;
		public static final int TesterPort = 3;
	}

	public static class RobotBounds {
		public static final double ForwardEdge = 0.438;
		public static final double BackwardEdge = -0.64; //-0.553;
		public static final double LeftEdge = 0.438;
		public static final double RightEdge = -0.438;
	}

	public static class VisionConstants {
		public static final boolean UseVisionForOdometry = true;
		public static final int[] AprilTagPipelines = {0, 1, 2, 3, 4};
		public static final double TxLaunchTolerance = 2;
		public static final double AprilTagAverageDistanceThresholdMeters = 3.5; // TODO: 2.5 used at week 1, but we should be to use a much higher value with confidence
		public static final double XMetersMidPoint = 8.25; //TODO Confirm
		public static final double RearCameraMountAngleRadians = Units.Degrees.of(30).in(Units.Radians);
		// Scale is currently in the range of [0,1]
		public static final double ConfidenceRequirement = 0.4;
		public static final double RobotSpeedThresholdMPS = 1.0;
	}

	public static class CANIdentifiers {
		public static final int FrontLeftSpeed = 30;
		public static final int FrontLeftAngle = 31;
		public static final int FrontLeftEncoder = 32;

		public static final int FrontRightSpeed = 33;
		public static final int FrontRightAngle = 34;
		public static final int FrontRightEncoder = 35;

		public static final int BackLeftSpeed = 36;
		public static final int BackLeftAngle = 37;
		public static final int BackLeftEncoder = 38;

		public static final int BackRightSpeed = 39;
		public static final int BackRightAngle = 40;
		public static final int BackRightEncoder = 41;

		public static final int IntakeUpper = 42;
		public static final int IntakeLower = 43;

		public static final int FeederMain = 44;

		public static final int LauncherTilt = 45;
		public static final int FlywheelLeft = 46;
		public static final	int FlywheelRight = 47;

		public static final int LiftLeft = 48;
		public static final int LiftRight = 49; 
		
		public static final int FeederTrap = 50;
	}

	public static class IOPorts {
		public static final int IntakeNoteSensor = 0;
		public static final int LiftBottomSensor = 9;
	}

	public static class SwerveConstants2024 {

		// Speeds
		public static final double MaxRobotSpeed_mps =  5.32; // 5.32 is maximum with a 6:1 (23:23 planet ratio)
		// public static final double MaxRobotSpeed_mps =  4.8; // 4.8 is maximum with a 6.55:1 (22:24 planet ratio)
		public static final double MaxRobotRotation_radps =  12.0; // 12.21157 is maximum with a 6.55:1 (22:24 planet ratio)
		public static final double SlowSpeedModifier = 0.65;
		public static final double DefaultSpeedModifier = 1.0;
		public static final double SuperSlowSpeedModifier = 0.25;

		public static final PIDValue DefaultTranslationPIDValues = new PIDValue(0.7, 0.0, 0.2);
		public static final double DefaultDriveToTargetTolerance_m = 0.03; 
		public static final PIDValue DefaultRotationPIDValues = new PIDValue(0.009, 0.0001, 0.0);
		public static final Rotation2d DefaultRotationTolerance = Rotation2d.fromDegrees(3);
		public static final PIDValue DefaultModuleAnglePIDValue = new PIDValue(60.0, 12.0, 0.0);
		public static final PIDFValue DefaultModuleVelocityPIDFValues = new PIDFValue(5.0, 0.0, 0.0, 2.19);
	}	

	public static class LauncherConstants {
		// public static final double TiltPotConversionFactor = 10.90146751;
		public static final double TiltAbsoluteEncoderConversionFactor = 90.151; // Old Value: 99.91
		public static final double TiltAbsoluteEncoderOffset = 73.75; // Old Value: 86.67
		public static final double TiltEncoderConversionFactor = 2.144329897;
		public static final Rotation2d TiltToleranceAngle = Rotation2d.fromDegrees(0.5);
		public static final double TiltRampRate = 0.1;
		public static final int TiltCurrentLimitAmps = 40;
		public static final Rotation2d MinAngle = Rotation2d.fromDegrees(6.65); // actual 5.65 - 6.65 is a safe limit
		public static final Rotation2d MaxAngle = Rotation2d.fromDegrees(58.2); // actual 59.2 - 58.2 is a safe limit
		public static final Rotation2d MinLaunchOverAngle = Rotation2d.fromDegrees(32);
		public static final Rotation2d MinLaunchOnClimb = Rotation2d.fromDegrees(16);

		public static final double TiltP = 0.068;
		public static final double TiltI = 0;
		public static final double TiltD = 0;

		public static final Rotation2d IntakeAngle = Rotation2d.fromDegrees(31.5);
		public static final Rotation2d SourceIntakeAngle = Rotation2d.fromDegrees(34.2);
		public static final Rotation2d LaunchSpitAngle = Rotation2d.fromDegrees(41.5);
		public static final Rotation2d PassNoteAngle = Rotation2d.fromDegrees(3);
		public static final Rotation2d AmpAngle = Rotation2d.fromDegrees(6.6);
		public static final Rotation2d BumperShotAngle = Rotation2d.fromDegrees(10);
		public static final Rotation2d TrapAngle = Rotation2d.fromDegrees(19.5);

		public static final double LauncherToleranceRPM = 150;
		public static final double DefaultLauncherSpeed = 1.0; // TODO Determine later
		public static final double DefaultLauncherAngle = 5;
		public static final double FlywheelRampRate = 0.1;
		public static final double FlywheelEncoderConversionFactor = 1;
		public static final double FlywheelP = 0.0007;
		public static final double FlywheelI = 0;
		public static final double FlywheelD = 0;
		public static final double FlywheelF = 0.00017;

		public static final double MaxRPM = 5500;

	}

	public static class LiftConstants {
		public static final double LiftEncoderConversionFactor = 71.81;
		public static final double LiftRampRate = 0.1;
		public static final double LiftP = 90;
		public static final double LiftI = 1;
		public static final double LiftD = 0;
		public static final double LiftG = 0.6;
		public static final double LiftToleranceMeters = 0.01;

		public static final double MinHeightMeters = 0.01;
		public static final double MaxHeightMeters = 0.693; 
		public static final double MaxSpeedBeforeBottom = 0.15;
		public static final double MaxSpeed = 0.3;

		public static final double DefaultLaunchMeters = 0.5;
		public static final double AmpMeters = 0.64; // 0.532
		public static final double SourceMeters = 0.3;

		public static final double DefaultHoldMeters = MinHeightMeters + 0.25;
		public static final double IntakeHeightMeters = MinHeightMeters;
		public static final double SourceIntakeHeightMeters = 0.38;
		public static final double LaunchSpitHeightMeters = MinHeightMeters;
		public static final double PassNoteHeight = 0.05;

		public static final double StartClimbHeight = MaxHeightMeters;

	}
}
