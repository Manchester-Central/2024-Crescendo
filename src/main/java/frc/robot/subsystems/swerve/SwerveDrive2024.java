package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AbsoluteEncoderConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AngleControllerConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.DriveConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.SpeedControllerConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveDrive2024 extends BaseSwerveDrive {
	private SwerveDrive2024(BaseSwerveModule[] modules, SwerveConfigs configs, Supplier<Rotation2d> getRotation) {

		super(modules, configs, getRotation);
	}

	public static SwerveDrive2024 createSwerveDrive() {
		// TODO do all the configs
		var configs = new SwerveConfigs()
			// Set max Speeds
			.setMaxRobotSpeed_mps(1.0 /*5.8*/) 
			.setMaxRobotRotation_radps(2.0 /*6.75*/) //TODO confirm
			// Translation PID settings
			.setDefaultTranslationPIDValues(new PIDValue(1.0, 0.0, 0.0)) //TODO confirm
			.setDefaultDriveToTargetTolerance(0.03) 
			// Rotation PID settings
			.setDefaultRotationPIDValues(new PIDValue(0.017, 0.0001, 0.0)) //TODO confirm
			.setDefaultRotationTolerance(Rotation2d.fromDegrees(3))
			// Module PID settings
			.setDefaultModuleAnglePIDValues(new PIDValue(48.0, 0.0, 0.0)) //TODO confirm
			.setDefaultModuleVelocityPIDFValues(new PIDFValue(5.0, 0.0, 0.0, 2.19)) //TODO confirm
			// field configs
			.setDefaultAlliance(Alliance.Blue) 
			// Debug/sim
			.setDebugMode(true)
			.setUpdateFrequency_hz(20);

		var speedGearRatio = 5.5; 
		var driveWheelDiameterMeters = 0.0991; 
		var wheelCircumferenceMeters = driveWheelDiameterMeters * Math.PI;
		var angleGearRatio = 144.0 / 14.0;
		var absoluteEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;
		var driverModeClosedLoopRampRatePeriod = 0.05;
		var driveToPositionClosedLoopRampRatePeriod = 0.65;
		var angleMotorDirection = InvertedValue.Clockwise_Positive;
		var leftSpeedMotorDirection = InvertedValue.CounterClockwise_Positive; //TODO confirm
		var rightSpeedMotorDirection = InvertedValue.Clockwise_Positive; //TODO confirm

		var frontLengthDistance = 0.282575;
		var backLengthDistance = 0.257175;
		//TODO create CAN_ID and absolute offset constants
		var frontLeftModule = new SwerveModule2024(
			"frontLeftModule",
			new Translation2d(frontLengthDistance, frontLengthDistance),
			new SpeedControllerConfig(30, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(31, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(32, absoluteEncoderDirection, Rotation2d.fromDegrees(-27.25)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);
		//TODO create CAN_ID and absolute offset constants
		var frontRightModule = new SwerveModule2024(
			"frontRightModule",
			new Translation2d(frontLengthDistance, -frontLengthDistance),
			new SpeedControllerConfig(33, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(34, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(35, absoluteEncoderDirection, Rotation2d.fromDegrees(-15.03)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		//TODO create CAN_ID and absolute offset constants
		var backLeftModule = new SwerveModule2024(
			"backLeftModule",
			new Translation2d(-backLengthDistance, backLengthDistance),
			new SpeedControllerConfig(36, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(37, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(38, absoluteEncoderDirection, Rotation2d.fromDegrees(43.24)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		//TODO create CAN_ID and absolute offset constants
		var backRightModule = new SwerveModule2024(
			"backRightModule",
			new Translation2d(-backLengthDistance, -backLengthDistance),
			new SpeedControllerConfig(39, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(40, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(41, absoluteEncoderDirection, Rotation2d.fromDegrees(64.95)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);

		BaseSwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
		var gyro = new AHRS(SPI.Port.kMXP);

		return new SwerveDrive2024(modules, configs, () -> gyro.getRotation2d());
	}
}
