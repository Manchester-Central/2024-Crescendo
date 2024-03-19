package frc.robot.subsystems.launcher;

import com.chaos131.logging.LogManager;
import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
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
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

	private CANSparkFlex m_flywheelLeft = new CANSparkFlex(CANIdentifiers.FlywheelLeft, MotorType.kBrushless);
	private CANSparkFlex m_flywheelRight = new CANSparkFlex(CANIdentifiers.FlywheelRight, MotorType.kBrushless);
	private PIDTuner m_flywheelPidTuner;

	private CANSparkFlex m_tiltController = new CANSparkFlex(CANIdentifiers.LauncherTilt, MotorType.kBrushless);
	// private SparkAnalogSensor m_tiltPot = m_tiltController.getAnalog(Mode.kAbsolute);
	private SparkAbsoluteEncoder m_absAngleEncoder;
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
		
		m_absAngleEncoder = m_tiltController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

		m_flywheelLeft.setIdleMode(IdleMode.kCoast);
		m_flywheelRight.setIdleMode(IdleMode.kCoast);
		m_flywheelPidTuner = new PIDTuner("Launcher/Flywheel", DebugConstants.LauncherDebugEnable, LauncherConstants.FlywheelP, LauncherConstants.FlywheelI, LauncherConstants.FlywheelD, LauncherConstants.FlywheelF, this::tuneFlywheelPID);
		m_flywheelLeft.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);
		m_flywheelRight.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);
		m_flywheelLeft.setClosedLoopRampRate(LauncherConstants.FlywheelRampRate);
		m_flywheelRight.setClosedLoopRampRate(LauncherConstants.FlywheelRampRate);
		m_flywheelLeft.setInverted(false);
		m_flywheelRight.setInverted(true);
		m_flywheelLeft.getPIDController().setOutputRange(0, 1);
		m_flywheelRight.getPIDController().setOutputRange(0, 1);

		// m_tiltPot.setInverted(true);
		// m_tiltPot.setPositionConversionFactor(LauncherConstants.TiltPotConversionFactor);
		m_absAngleEncoder.setInverted(true);
		m_absAngleEncoder.setPositionConversionFactor(LauncherConstants.TiltAbsoluteEncoderConversionFactor);
		m_absAngleEncoder.setZeroOffset(LauncherConstants.TiltAbsoluteEncoderOffset);
		m_tiltController.getEncoder().setPositionConversionFactor(LauncherConstants.TiltEncoderConversionFactor);
		m_tiltController.setIdleMode(IdleMode.kCoast);
		// m_tiltController.getPIDController().setFeedbackDevice(m_tiltPot);
		m_tiltController.getPIDController().setFeedbackDevice(m_absAngleEncoder);
		m_tiltController.setClosedLoopRampRate(LauncherConstants.TiltRampRate);
		m_tiltController.setInverted(true);
		m_tiltController.setSmartCurrentLimit(LauncherConstants.TiltCurrentLimitAmps);
		// m_tiltController.getEncoder().setPosition(LauncherConstants.MaxAngle.getDegrees()); // TODO: Remove when abs angle is working again
		m_tiltPIDTuner = new PIDTuner("Launcher/Tilt", DebugConstants.LauncherDebugEnable, LauncherConstants.TiltP, LauncherConstants.TiltI, LauncherConstants.TiltD, this::tuneTiltPID);
		
		
		m_flywheelLeft.burnFlash();
		m_flywheelRight.burnFlash();
		m_tiltController.burnFlash();
		// recalibrateTilt(); // ?? TODO: add back in when Abs angle is working again (but check problem with syncing on start up)

		var logManager = LogManager.getInstance();
		logManager.addNumber("Launcher/LeftRPM", DebugConstants.LauncherDebugEnable, () -> m_flywheelLeft.getEncoder().getVelocity());
		logManager.addNumber("Launcher/RightRPM", DebugConstants.LauncherDebugEnable, () -> m_flywheelRight.getEncoder().getVelocity());
		logManager.addNumber("Launcher/TargetRPM", DebugConstants.LauncherDebugEnable, () -> m_targetRPM);
		logManager.addNumber("Launcher/LeftCurrentAmps", DebugConstants.LauncherDebugEnable, () -> m_flywheelLeft.getOutputCurrent());
		logManager.addNumber("Launcher/RightCurrentAmps", DebugConstants.LauncherDebugEnable, () -> m_flywheelRight.getOutputCurrent());
		logManager.addNumber("Launcher/LeftAppliedOutput", DebugConstants.LauncherDebugEnable, () -> m_flywheelLeft.getAppliedOutput());
		logManager.addNumber("Launcher/RightAppliedOutput", DebugConstants.LauncherDebugEnable, () -> m_flywheelRight.getAppliedOutput());
		logManager.addNumber("Launcher/LeftError", DebugConstants.LauncherDebugEnable, () -> m_targetRPM - m_flywheelLeft.getEncoder().getVelocity());
		logManager.addNumber("Launcher/RightError", DebugConstants.LauncherDebugEnable, () -> m_targetRPM - m_flywheelRight.getEncoder().getVelocity());

		// logManager.addNumber("Launcher/AngleDegrees", true, () -> getCurrentAngle().getDegrees());
		logManager.addNumber("Launcher/AbsAngleDegrees", true, () -> getAbsoluteTiltAngle().getDegrees());
		logManager.addNumber("Launcher/TargetAngleDegrees", DebugConstants.LauncherDebugEnable, () -> m_targetAngle.getDegrees());
		logManager.addNumber("Launcher/TiltAppliedOutput", DebugConstants.LauncherDebugEnable, () -> m_tiltController.getAppliedOutput());
		logManager.addNumber("Launcher/TiltCurrentAmps", DebugConstants.LauncherDebugEnable, () -> m_tiltController.getOutputCurrent());
		logManager.addNumber("Launcher/TiltRawPosition", DebugConstants.LauncherDebugEnable, () -> m_tiltController.getEncoder().getPosition());

		
	}

	public void setTiltSpeed(double speed) {
		if (getAbsoluteTiltAngle().getDegrees() < LauncherConstants.MinAngle.getDegrees()) {
			speed = MathUtil.clamp(speed, 0, 1);
		} else if (getAbsoluteTiltAngle().getDegrees() > LauncherConstants.MaxAngle.getDegrees()) {
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

	public void setLauncherRPM(double leftSpeedRPM, double rightSpeedRPM) {
		m_targetRPM = leftSpeedRPM;
		

		if (Robot.isSimulation()) {
			// Slowly adjust the power to make the simulator show the launcher getting up to speed
			m_simFlywheelPower += MathUtil.clamp(m_simFlywheelPid.calculate(m_simFlywheelRPM, m_targetRPM), -m_simMaxFlywheelPowerChangePerLoop, m_simMaxFlywheelPowerChangePerLoop);
		}
		m_flywheelLeft.getPIDController().setReference(leftSpeedRPM, ControlType.kVelocity);
		m_flywheelRight.getPIDController().setReference(rightSpeedRPM, ControlType.kVelocity);
	}

	public double getRightLauncherRPM() {
		if (Robot.isSimulation()) {
			return m_simFlywheelRPM;
		}
		return m_flywheelRight.getEncoder().getVelocity();
	}

	public double getLeftLauncherRPM() {
		if (Robot.isSimulation()) {
			return m_simFlywheelRPM;
		}
		return m_flywheelLeft.getEncoder().getVelocity();
	}

	public boolean atTargetRPM(double leftTargetRPM, double rightTargetRPM) {
		return Math.abs(getRightLauncherRPM() - rightTargetRPM) <= LauncherConstants.LauncherToleranceRPM 
			&& Math.abs(getLeftLauncherRPM() - leftTargetRPM) <= LauncherConstants.LauncherToleranceRPM; 
	}

	/**
	 * Gets the current angle of the launcher
	 */
	// public Rotation2d getCurrentAngle() {
	// 	if (Robot.isSimulation()) {
	// 		return m_simAngle;
	// 	}
	// 	return Rotation2d.fromDegrees(m_tiltController.getEncoder().getPosition());
	// }

	public boolean atTargetAngle(Rotation2d targetAngle) {
		return Math.abs(getAbsoluteTiltAngle().minus(targetAngle).getDegrees()) <= LauncherConstants.TiltToleranceAngle.getDegrees();
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
		if (Robot.isSimulation()) {
			return m_simAngle;
		}
		// return Rotation2d.fromDegrees(m_tiltPot.getPosition());
		return Rotation2d.fromDegrees(m_absAngleEncoder.getPosition());
	}

	public void recalibrateTilt() {
        m_tiltController.getEncoder().setPosition(0); // TODO Don't Do This
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
		m_flywheelLeft.getPIDController().setFF(pidValue.F * 1.1); // old value 0.00017
		m_flywheelRight.getPIDController().setP(pidValue.P);
		m_flywheelRight.getPIDController().setI(pidValue.I);
		m_flywheelRight.getPIDController().setD(pidValue.D);
		m_flywheelRight.getPIDController().setFF(pidValue.F * 1.04);
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
	}
}
