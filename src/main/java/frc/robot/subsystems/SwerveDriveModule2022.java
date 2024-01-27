package frc.robot.subsystems;

import com.chaos131.swerve.implementation.TalonFxAndCancoderSwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule2022 extends TalonFxAndCancoderSwerveModule {

	public SwerveDriveModule2022(String name, Translation2d translation, SpeedControllerConfig speedControllerConfig,
			AngleControllerConfig angleControllerConfig, AbsoluteEncoderConfig absoluteEncoderConfig,
			DriveConfig driveConfig) {
		super(name, translation, speedControllerConfig, angleControllerConfig, absoluteEncoderConfig, driveConfig);

	}
	
	@Override
	public void updateDashboard() {
		super.updateDashboard();
		SmartDashboard.putNumber(getDSKey("absoluteAngleDegrees"), getAbsoluteAngle().getDegrees());
		SmartDashboard.putNumber(getDSKey("motorAngleDegrees"), getEncoderAngle().getDegrees());
		SmartDashboard.putNumber(getDSKey("currentMPS"), getEncoderVelocity_mps());
		SmartDashboard.putNumber(getDSKey("targetMPS"), m_speedController.getClosedLoopReference().getValueAsDouble());

	}
}
