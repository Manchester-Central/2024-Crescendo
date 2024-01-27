package frc.robot.subsystems;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.SwerveConfigs;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AbsoluteEncoderConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.AngleControllerConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.DriveConfig;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule.SpeedControllerConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive2022 extends BaseSwerveDrive {
	private SwerveDrive2022(SwerveConfigs configs) {
		
		super(null, configs, null);
	}

	public static SwerveDrive2022 createSwerveDrive() {
		var configs = new SwerveConfigs();

		var halfWidthMeters = -0.2957;
    	var halfLengthMeters = 0.32067;

		var speedGearRatio = 7.80;
		var driveWheelWidthMeters = 0.092;
		var wheelCircumferenceMeters = driveWheelWidthMeters * Math.PI;
		var angleGearRatio = 144.0 / 14.0;
		var absoluteEncoderDirection = SensorDirectionValue.Clockwise_Positive;
		var driverModeClosedLoopRampRatePeriod = 0;
        var driveToPositionClosedLoopRampRatePeriod = 0;
		var angleMotorDirection = InvertedValue.Clockwise_Positive;
		var leftSpeedMotorDirection = InvertedValue.CounterClockwise_Positive;
        var rightSpeedMotorDirection = InvertedValue.Clockwise_Positive;

		var frontLeftModule = new TalonFxAndCancoderSwerveModule(
			"frontLeftModule",
			new Translation2d(halfLengthMeters, -halfWidthMeters),
			new SpeedControllerConfig(6, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(5, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(22, absoluteEncoderDirection, Rotation2d.fromDegrees(109.2)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);
		var frontRightModule = new TalonFxAndCancoderSwerveModule(
			"frontRightModule",
			new Translation2d(halfLengthMeters, halfWidthMeters),
			new SpeedControllerConfig(8, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(4, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(21, absoluteEncoderDirection, Rotation2d.fromDegrees(5.9)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		var backLeftModule = new TalonFxAndCancoderSwerveModule(
			"backLeftModule",
			new Translation2d(-halfLengthMeters, -halfWidthMeters),
			new SpeedControllerConfig(3, leftSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(7, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(23, absoluteEncoderDirection, Rotation2d.fromDegrees(123.3)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(-45))
		);
		var backRightModule = new TalonFxAndCancoderSwerveModule(
			"backRightModule",
			new Translation2d(-halfLengthMeters, halfWidthMeters),
			new SpeedControllerConfig(2, rightSpeedMotorDirection, speedGearRatio, wheelCircumferenceMeters),
			new AngleControllerConfig(1, angleMotorDirection, angleGearRatio),
			new AbsoluteEncoderConfig(20, absoluteEncoderDirection, Rotation2d.fromDegrees(-164)),
			new DriveConfig(driverModeClosedLoopRampRatePeriod, driveToPositionClosedLoopRampRatePeriod, Rotation2d.fromDegrees(45))
		);


		return new SwerveDrive2022(configs);
	}
}
