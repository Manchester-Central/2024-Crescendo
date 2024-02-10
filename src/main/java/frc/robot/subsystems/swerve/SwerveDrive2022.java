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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Vision;

public class SwerveDrive2022 extends BaseSwerveDrive {
	private Vision m_vision;
	private SwerveDrive2022(BaseSwerveModule[] modules, SwerveConfigs configs, Supplier<Rotation2d> getRotation) {

		super(modules, configs, getRotation);
	}

	public static SwerveDrive2022 createSwerveDrive() {
		var configs = new SwerveConfigs()
			// Set max Speeds
			.setMaxRobotSpeed_mps(3.8)
			.setMaxRobotRotation_radps(6.75)
			// Translation PID settings
			.setDefaultTranslationPIDValues(new PIDValue(1.0, 0.0, 0.0))
			.setDefaultDriveToTargetTolerance(0.03)
			// Rotation PID settings
			.setDefaultRotationPIDValues(new PIDValue(0.01, 0.0001, 0.0))
			.setDefaultRotationTolerance(Rotation2d.fromDegrees(3))
			// Module PID settings
			.setDefaultModuleAnglePIDValues(new PIDValue(60.0, 2, 0.0))
			.setDefaultModuleVelocityPIDFValues(new PIDFValue(10.0, 0.0, 0.0, 3.35))
			// field configs
			.setDefaultAlliance(Alliance.Blue)
			// Debug/sim
			.setDebugMode(true)
			.setUpdateFrequency_hz(20);

		var halfWidthMeters = 0.2957;
		var halfLengthMeters = 0.32067;

		var speedGearRatio = 7.80;
		var driveWheelDiameterMeters = 0.092;
		var wheelCircumferenceMeters = driveWheelDiameterMeters * Math.PI;
		var angleGearRatio = 144.0 / 14.0;
		var absoluteEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;
		var driverModeClosedLoopRampRatePeriod = 0.2;
		var driveToPositionClosedLoopRampRatePeriod = 0.65;
		var angleMotorDirection = InvertedValue.Clockwise_Positive;
		var leftSpeedMotorDirection = InvertedValue.CounterClockwise_Positive;
		var rightSpeedMotorDirection = InvertedValue.Clockwise_Positive;

		var frontLeftModule = new SwerveModule2022(
			"frontLeftModule",
			new Translation2d(halfLengthMeters, halfWidthMeters),
			new SpeedControllerConfig(6, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(5, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(22, absoluteEncoderDirection, Rotation2d.fromDegrees(110.039 /*109.2*/)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);
		var frontRightModule = new SwerveModule2022(
			"frontRightModule",
			new Translation2d(halfLengthMeters, -halfWidthMeters),
			new SpeedControllerConfig(8, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(4, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(21, absoluteEncoderDirection, Rotation2d.fromDegrees(6.064 /*5.9*/)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		var backLeftModule = new SwerveModule2022(
			"backLeftModule",
			new Translation2d(-halfLengthMeters, halfWidthMeters),
			new SpeedControllerConfig(3, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(7, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(23, absoluteEncoderDirection, Rotation2d.fromDegrees(123.837 /*123.3*/)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		var backRightModule = new SwerveModule2022(
			"backRightModule",
			new Translation2d(-halfLengthMeters, -halfWidthMeters),
			new SpeedControllerConfig(2, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(1, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(20, absoluteEncoderDirection, Rotation2d.fromDegrees(-165.058 /*-164*/)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);

		BaseSwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
		var gyro = new AHRS(SPI.Port.kMXP);

		return new SwerveDrive2022(modules, configs, () -> gyro.getRotation2d());
	}


	@Override
	public void periodic() {
		super.periodic();
		m_vision.periodic();
		if(m_vision.getPose() != null) {
			addVisionMeasurement(m_vision.getPose(), m_vision.getLatencySeconds());
		}
		SmartDashboard.putNumber("Odometry Angle Degrees", getOdometryRotation().getDegrees());

	
	} 

	public void setVision(Vision system) {
	
		m_vision = system;
	
	}


}
