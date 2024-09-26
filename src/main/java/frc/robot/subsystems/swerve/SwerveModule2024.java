package frc.robot.subsystems.swerve;

import com.chaos131.logging.LogManager;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DebugConstants;

public class SwerveModule2024 extends TalonFxAndCancoderSwerveModule {
	public SwerveModule2024(String name, Translation2d translation, SpeedControllerConfig speedControllerConfig,
			AngleControllerConfig angleControllerConfig, AbsoluteEncoderConfig absoluteEncoderConfig,
			DriveConfig driveConfig) {
		super(name, translation, speedControllerConfig, angleControllerConfig, absoluteEncoderConfig, driveConfig);
		m_speedConfig.CurrentLimits = new CurrentLimitsConfigs();
		m_speedConfig.CurrentLimits.SupplyCurrentLimit = 35;
		m_speedConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
		m_speedConfig.CurrentLimits.SupplyCurrentThreshold = 85;
		m_speedConfig.CurrentLimits.SupplyTimeThreshold = 0.01;
		m_speedConfig.CurrentLimits.StatorCurrentLimit = 75;
		m_speedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		m_speedController.getConfigurator().apply(m_speedConfig.CurrentLimits);
		m_speedController.setPosition(0);
		var logManager = LogManager.getInstance();
		logManager.addNumber(getDSKey("currentMPS"), DebugConstants.DriveDebugEnable, () -> getEncoderVelocity_mps());
		logManager.addNumber(getDSKey("motorAngleDegrees"), DebugConstants.DriveDebugEnable, () -> getEncoderAngle().getDegrees());
		logManager.addNumber(getDSKey("absoluteAngleDegrees"), DebugConstants.DriveDebugEnable, () -> getAbsoluteAngle().getDegrees());
		logManager.addNumber(getDSKey("targetMPS"), DebugConstants.DriveDebugEnable, () -> m_speedController.getClosedLoopReference().getValueAsDouble());
		logManager.addNumber(getDSKey("targetDegrees"), DebugConstants.DriveDebugEnable, () -> m_angleController.getClosedLoopReference().getValueAsDouble());
		logManager.addNumber(getDSKey("distanceTraveledMeters"), DebugConstants.DriveDebugEnable, () -> getEncoderDistance_m());
		logManager.addNumber(getDSKey("errorAngleDegrees"), DebugConstants.DriveDebugEnable, () -> Rotation2d.fromRotations(m_angleController.getClosedLoopError().getValueAsDouble()).getDegrees());
		logManager.addNumber(getDSKey("angleCurrentAmps"), DebugConstants.DriveDebugEnable, () -> m_angleController.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocitySupplyCurrentAmps"), DebugConstants.DriveDebugEnable, () -> m_speedController.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocityStatorCurrentAmps"), DebugConstants.DriveDebugEnable, () -> m_speedController.getStatorCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocityMotorVoltage"), DebugConstants.DriveDebugEnable, () -> m_speedController.getMotorVoltage().getValueAsDouble());
		logManager.addNumber(getDSKey("angleMotorVoltage"), DebugConstants.DriveDebugEnable, () -> m_angleController.getMotorVoltage().getValueAsDouble());
		
		// "I love dark mode" - Josh 02/08/2024
	}

	public void setPercentSpeed(double percentSpeed) {
		m_speedController.set(percentSpeed);
	}
}
