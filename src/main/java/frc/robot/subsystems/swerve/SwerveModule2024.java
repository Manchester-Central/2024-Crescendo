package frc.robot.subsystems.swerve;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule2024 extends TalonFxAndCancoderSwerveModule {
	public SwerveModule2024(String name, Translation2d translation, SpeedControllerConfig speedControllerConfig,
			AngleControllerConfig angleControllerConfig, AbsoluteEncoderConfig absoluteEncoderConfig,
			DriveConfig driveConfig) {
		super(name, translation, speedControllerConfig, angleControllerConfig, absoluteEncoderConfig, driveConfig);
		m_speedConfig.CurrentLimits = new CurrentLimitsConfigs();
		m_speedConfig.CurrentLimits.SupplyCurrentLimit = 70;
		m_speedConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
		m_speedController.getConfigurator().apply(m_speedConfig.CurrentLimits);
		m_speedController.setPosition(0);
		// "I love dark mode" - Josh 02/08/2024
	}
	
	@Override
	public void updateDashboard() {
		super.updateDashboard();
		SmartDashboard.putNumber(getDSKey("absoluteAngleDegrees"), getAbsoluteAngle().getDegrees());
		SmartDashboard.putNumber(getDSKey("motorAngleDegrees"), getEncoderAngle().getDegrees());
		SmartDashboard.putNumber(getDSKey("currentMPS"), getEncoderVelocity_mps());
		SmartDashboard.putNumber(getDSKey("targetMPS"), m_speedController.getClosedLoopReference().getValueAsDouble());
		SmartDashboard.putNumber(getDSKey("targetDegrees"), m_angleController.getClosedLoopReference().getValueAsDouble());
		SmartDashboard.putNumber(getDSKey("distanceTraveledMeters"), getEncoderDistance_m());
		SmartDashboard.putNumber(getDSKey("errorAngleDegrees"), Rotation2d.fromRotations(m_angleController.getClosedLoopError().getValueAsDouble()).getDegrees());
	}

	public void setPercentSpeed(double percentSpeed) {
		m_speedController.set(percentSpeed);
	}
}
