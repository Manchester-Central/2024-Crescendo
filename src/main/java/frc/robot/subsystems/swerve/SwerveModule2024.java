package frc.robot.subsystems.swerve;

import com.chaos131.logging.LogManager;
import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule2024 extends TalonFxAndCancoderSwerveModule {
	public SwerveModule2024(String name, Translation2d translation, SpeedControllerConfig speedControllerConfig,
			AngleControllerConfig angleControllerConfig, AbsoluteEncoderConfig absoluteEncoderConfig,
			DriveConfig driveConfig) {
		super(name, translation, speedControllerConfig, angleControllerConfig, absoluteEncoderConfig, driveConfig);
		m_speedConfig.CurrentLimits = new CurrentLimitsConfigs();
		m_speedConfig.CurrentLimits.SupplyCurrentLimit = 70;
		m_speedConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
		m_speedConfig.CurrentLimits.StatorCurrentLimit = 35;
		m_speedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		m_speedController.getConfigurator().apply(m_speedConfig.CurrentLimits);
		m_speedController.setPosition(0);
		var logManager = LogManager.getInstance();
		logManager.addNumber(getDSKey("currentMPS"), Constants.DebugMode, () -> getEncoderVelocity_mps());
		logManager.addNumber(getDSKey("motorAngleDegrees"), Constants.DebugMode, () -> getEncoderAngle().getDegrees());
		logManager.addNumber(getDSKey("absoluteAngleDegrees"), Constants.DebugMode, () -> getAbsoluteAngle().getDegrees());
		logManager.addNumber(getDSKey("targetMPS"), Constants.DebugMode, () -> m_speedController.getClosedLoopReference().getValueAsDouble());
		logManager.addNumber(getDSKey("targetDegrees"), Constants.DebugMode, () -> m_angleController.getClosedLoopReference().getValueAsDouble());
		logManager.addNumber(getDSKey("distanceTraveledMeters"), Constants.DebugMode, () -> getEncoderDistance_m());
		logManager.addNumber(getDSKey("errorAngleDegrees"), Constants.DebugMode, () -> Rotation2d.fromRotations(m_angleController.getClosedLoopError().getValueAsDouble()).getDegrees());
		logManager.addNumber(getDSKey("angleCurrentAmps"), Constants.DebugMode, () -> m_angleController.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocitySupplyCurrentAmps"), Constants.DebugMode, () -> m_speedController.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocityStatorCurrentAmps"), Constants.DebugMode, () -> m_speedController.getStatorCurrent().getValueAsDouble());
		logManager.addNumber(getDSKey("velocityMotorVoltage"), Constants.DebugMode, () -> m_speedController.getMotorVoltage().getValueAsDouble());
		logManager.addNumber(getDSKey("angleMotorVoltage"), Constants.DebugMode, () -> m_angleController.getMotorVoltage().getValueAsDouble());
		
		// "I love dark mode" - Josh 02/08/2024
	}

	public void setPercentSpeed(double percentSpeed) {
		m_speedController.set(percentSpeed);
	}
}
