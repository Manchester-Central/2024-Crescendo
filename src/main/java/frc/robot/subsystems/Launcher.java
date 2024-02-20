package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

	private CANSparkFlex m_flywheelLeft = new CANSparkFlex(CANIdentifiers.FlywheelLeft, MotorType.kBrushless);
	private CANSparkFlex m_flywheelRight = new CANSparkFlex(CANIdentifiers.FlywheelRight, MotorType.kBrushless);
	private PIDTuner m_flywheelPidTuner;

	private CANSparkFlex m_tiltController = new CANSparkFlex(CANIdentifiers.LauncherTilt, MotorType.kBrushless);
	private SparkAnalogSensor m_tiltPot = m_tiltController.getAnalog(Mode.kAbsolute);
	private PIDTuner m_tiltPIDTuner;

	//Target values
	private double m_targetRPM = 0;
	private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

	// Angle Sim
	private Rotation2d m_simAngle = Rotation2d.fromDegrees(0);
	private double m_simAnglePower = 0;
	private double m_simMaxDegreesChangePerLoop = 1;
	private PIDController m_simAnglePid = new PIDController(1, 0, 0);


	// Flywheel sim
	private double m_simFlywheelRPM = 0;
	private double m_simFlywheelPower = 0;
	private double m_simMaxFlywheelPowerChangePerLoop = 0.05;
	private PIDController m_simFlywheelPid = new PIDController(1, 0, 0);

	public Launcher() {
		m_flywheelLeft.restoreFactoryDefaults();
		m_flywheelRight.restoreFactoryDefaults();
		m_tiltController.restoreFactoryDefaults();
		
		m_flywheelLeft.setIdleMode(IdleMode.kCoast);
		m_flywheelRight.setIdleMode(IdleMode.kCoast);
		m_flywheelPidTuner = new PIDTuner("Launcher/Flywheel", Constants.DebugMode, LauncherConstants.FlywheelP, LauncherConstants.FlywheelI, LauncherConstants.FlywheelD, LauncherConstants.FlywheelF, this::tuneFlywheelPID);
		m_flywheelLeft.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);
		m_flywheelRight.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);

		m_flywheelLeft.setInverted(false);
		m_flywheelRight.setInverted(true);

		m_tiltPot.setInverted(true);
		m_tiltPot.setPositionConversionFactor(LauncherConstants.TiltPotConversionFactor);
		m_tiltController.getEncoder().setPositionConversionFactor(LauncherConstants.TiltEncoderConversionFactor);
		m_tiltController.setIdleMode(IdleMode.kCoast);
		m_tiltController.getPIDController().setFeedbackDevice(m_tiltPot);
		m_tiltController.setInverted(true);
		m_tiltController.getEncoder().setPosition(LauncherConstants.MaxAngle.getDegrees()); // TODO: Remove when abs angle is working again
		m_tiltPIDTuner = new PIDTuner("Launcher/Tilt", Constants.DebugMode, LauncherConstants.TiltP, LauncherConstants.TiltI, LauncherConstants.TiltD, this::tuneTiltPID);
		

		m_flywheelLeft.burnFlash();
		m_flywheelRight.burnFlash();
		m_tiltController.burnFlash();
		//recalibrateTilt(); ?? TODO: add back in when Abs angle is working again (but check problem with syncing on start up)
	}

	public void setTiltSpeed(double speed) {
		if (getCurrentAngle().getDegrees() < LauncherConstants.MinAngle.getDegrees()) {
			speed = MathUtil.clamp(speed, 0, 1);
		} else if (getCurrentAngle().getDegrees() > LauncherConstants.MaxAngle.getDegrees()) {
			speed = MathUtil.clamp(speed, -1, 0);
		} 
		m_simAnglePower = speed;
		m_tiltController.set(speed);
	}

	/**
	 * Sets the target angle for the launcher
	 * @param angle the target angle to move to
	 */
	public void setTiltAngle(Rotation2d angle) {
		angle = Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), LauncherConstants.MinAngle.getDegrees(), LauncherConstants.MaxAngle.getDegrees()));
		m_targetAngle = angle;
		if (Robot.isSimulation()) {
			m_simAnglePower = MathUtil.clamp(m_simAnglePid.calculate(m_simAngle.getDegrees(), angle.getDegrees()), -1.0, 1.0);
		}
		m_tiltController.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
	}

	public void setLauncherRPM(double speedRPM) {
		m_targetRPM = speedRPM;

		if (Robot.isSimulation()) {
			// Slowly adjust the power to make the simulator show the launcher getting up to speed
			m_simFlywheelPower += MathUtil.clamp(m_simFlywheelPid.calculate(m_simFlywheelRPM, speedRPM), -m_simMaxFlywheelPowerChangePerLoop, m_simMaxFlywheelPowerChangePerLoop);
		}
		m_flywheelLeft.getPIDController().setReference(speedRPM, ControlType.kVelocity);
		m_flywheelRight.getPIDController().setReference(speedRPM, ControlType.kVelocity);
	}

	public double getLauncherRPM() {
		if (Robot.isSimulation()) {
			return m_simFlywheelRPM;
		}
		return m_flywheelRight.getEncoder().getVelocity();
	}

	public boolean atTargetRPM(double targetRPM) {
		return Math.abs(getLauncherRPM() - targetRPM) <= LauncherConstants.LauncherToleranceRPM;
	}

	/**
	 * Gets the current angle of the launcher
	 */
	public Rotation2d getCurrentAngle() {
		if (Robot.isSimulation()) {
			return m_simAngle;
		}
		return Rotation2d.fromDegrees(m_tiltController.getEncoder().getPosition());
	}

	public boolean atTargetAngle(Rotation2d targetAngle) {
		return Math.abs(getCurrentAngle().minus(targetAngle).getDegrees()) <= LauncherConstants.TiltToleranceAngle.getDegrees();
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setLauncherPower(double power) {
		if (Robot.isSimulation()) {
			m_simFlywheelPower = power;
		}
		m_flywheelLeft.set(power);
		m_flywheelRight.set(power);
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the launcher
	 */
	public double getCurrentLauncherPower() {
		if (Robot.isSimulation()) {
			return m_simFlywheelPower;
		}
		return m_flywheelRight.get();
	}

	public Rotation2d getAbsoluteTiltAngle() {
		return Rotation2d.fromDegrees(m_tiltPot.getPosition());
	}

	public void recalibrateTilt() {
        m_tiltController.getEncoder().setPosition(getAbsoluteTiltAngle().getDegrees());
    }

	public void tuneTiltPID(PIDFValue pidValue){
        m_tiltController.getPIDController().setP(pidValue.P);
        m_tiltController.getPIDController().setI(pidValue.I);        
        m_tiltController.getPIDController().setD(pidValue.D);
    }

	public void tuneFlywheelPID(PIDFValue pidValue) {
		m_flywheelLeft.getPIDController().setP(pidValue.P);
		m_flywheelLeft.getPIDController().setI(pidValue.I);
		m_flywheelLeft.getPIDController().setD(pidValue.D);
		m_flywheelLeft.getPIDController().setFF(pidValue.F);
		m_flywheelRight.getPIDController().setP(pidValue.P);
		m_flywheelRight.getPIDController().setI(pidValue.I);
		m_flywheelRight.getPIDController().setD(pidValue.D);
		m_flywheelRight.getPIDController().setFF(pidValue.F);
	}

	@Override
	public void periodic() {
		super.periodic();
		m_tiltPIDTuner.tune();
		m_flywheelPidTuner.tune();

		if (Robot.isSimulation()) {
			m_simAngle = m_simAngle.plus(Rotation2d.fromDegrees(m_simAnglePower * m_simMaxDegreesChangePerLoop));
			m_simFlywheelRPM = m_simFlywheelPower * LauncherConstants.MaxRPM;
		}

		SmartDashboard.putNumber("Launcher/LeftRPM", m_flywheelLeft.getEncoder().getVelocity());
		SmartDashboard.putNumber("Launcher/RightRPM", m_flywheelRight.getEncoder().getVelocity());
		SmartDashboard.putNumber("Launcher/AngleDegrees", getCurrentAngle().getDegrees());
		SmartDashboard.putNumber("Launcher/AbsAngleDegrees", getAbsoluteTiltAngle().getDegrees());
		SmartDashboard.putNumber("Launcher/TargetRPM", m_targetRPM);
		SmartDashboard.putNumber("Launcher/TargetAngleDegrees", m_targetAngle.getDegrees());
	}
}
