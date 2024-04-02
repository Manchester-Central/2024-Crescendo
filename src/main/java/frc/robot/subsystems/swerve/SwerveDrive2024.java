package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AbsoluteEncoderConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AngleControllerConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.DriveConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.SpeedControllerConfig;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.SwerveConstants2024;

public class SwerveDrive2024 extends SwerveDrive {
	private Pigeon2 m_gyro;

	private SwerveDrive2024(BaseSwerveModule[] modules, SwerveConfigs configs, Pigeon2 gyro) {

		super(modules, configs, () -> gyro.getRotation2d());
		m_gyro = gyro;
	}

	public static SwerveDrive2024 createSwerveDrive() {
		// TODO do all the configs
		var configs = new SwerveConfigs()
			// Set max Speeds
			.setMaxRobotSpeed_mps(SwerveConstants2024.MaxRobotSpeed_mps) 
			.setMaxRobotRotation_radps(SwerveConstants2024.MaxRobotRotation_radps) //TODO confirm
			// Translation PID settings
			.setDefaultTranslationPIDValues(SwerveConstants2024.DefaultTranslationPIDValues) //TODO confirm
			.setDefaultDriveToTargetTolerance(SwerveConstants2024.DefaultDriveToTargetTolerance_m) 
			// Rotation PID settings
			.setDefaultRotationPIDValues(SwerveConstants2024.DefaultRotationPIDValues) //TODO confirm
			.setDefaultRotationTolerance(SwerveConstants2024.DefaultRotationTolerance)
			// Module PID settings
			.setDefaultModuleAnglePIDValues(SwerveConstants2024.DefaultModuleAnglePIDValue) //TODO confirm
			.setDefaultModuleVelocityPIDFValues(SwerveConstants2024.DefaultModuleVelocityPIDFValues) //TODO confirm
			// field configs
			.setDefaultAlliance(Constants.DefaultAlliance) 
			// Debug/sim
			.setDebugMode(DebugConstants.DriveDebugEnable)
			.setUpdateFrequency_hz(Constants.SimUpdateFrequency);

		var speedGearRatio = 6.55; //5.5 
		var driveWheelDiameterMeters = 0.1016; // 0.0991 
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
			new SpeedControllerConfig(CANIdentifiers.FrontLeftSpeed, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(CANIdentifiers.FrontLeftAngle, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(CANIdentifiers.FrontLeftEncoder, absoluteEncoderDirection, Rotation2d.fromDegrees(26.19 + 90)), //-0.61 + 90
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);
		//TODO create CAN_ID and absolute offset constants
		var frontRightModule = new SwerveModule2024(
			"frontRightModule",
			new Translation2d(frontLengthDistance, -frontLengthDistance),
			new SpeedControllerConfig(CANIdentifiers.FrontRightSpeed, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(CANIdentifiers.FrontRightAngle, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(CANIdentifiers.FrontRightEncoder, absoluteEncoderDirection, Rotation2d.fromDegrees(75.06 - 90)), // -70.57 - 90
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		//TODO create CAN_ID and absolute offset constants
		var backLeftModule = new SwerveModule2024(
			"backLeftModule",
			new Translation2d(-backLengthDistance, backLengthDistance),
			new SpeedControllerConfig(CANIdentifiers.BackLeftSpeed, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(CANIdentifiers.BackLeftAngle, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(CANIdentifiers.BackLeftEncoder, absoluteEncoderDirection, Rotation2d.fromDegrees(-100.8 + 90)), // 12.57 + 90
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		//TODO create CAN_ID and absolute offset constants
		var backRightModule = new SwerveModule2024(
			"backRightModule",
			new Translation2d(-backLengthDistance, -backLengthDistance),
			new SpeedControllerConfig(CANIdentifiers.BackRightSpeed, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(CANIdentifiers.BackRightAngle, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(CANIdentifiers.BackRightEncoder, absoluteEncoderDirection, Rotation2d.fromDegrees(-46.05 - 90)), // -37 - 90
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);

		BaseSwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
		var gyro = new Pigeon2(CANIdentifiers.PigeonGyro);

		return new SwerveDrive2024(modules, configs, gyro);
	}

	// public void testModuleSpeed(double percentSpeed) {
	// 	forAllModules(module -> ((SwerveModule2024)module).setPercentSpeed(percentSpeed));
	// }
}
