package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

	private Rotation2d simAngle = Rotation2d.fromDegrees(0);
	private double simPower = 0;

	public Launcher() {
		m_flywheelLeft.setIdleMode(IdleMode.kCoast);
		m_flywheelRight.setIdleMode(IdleMode.kCoast);
		m_flywheelPidTuner = new PIDTuner("Launcher/Flywheel", Constants.DebugMode, LauncherConstants.FlywheelP, LauncherConstants.FlywheelI, LauncherConstants.FlywheelD, LauncherConstants.FlywheelF, this::tuneFlywheelPID);
		m_flywheelLeft.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);
		m_flywheelRight.getEncoder().setVelocityConversionFactor(LauncherConstants.FlywheelEncoderConversionFactor);

		m_tiltPot.setPositionConversionFactor(LauncherConstants.TiltPotConversionFactor);
		m_tiltController.getEncoder().setPositionConversionFactor(LauncherConstants.TiltEncoderConversionFactor);
		m_tiltController.setIdleMode(IdleMode.kBrake);
		m_tiltPIDTuner = new PIDTuner("Launcher/Tilt", Constants.DebugMode, LauncherConstants.TiltP, LauncherConstants.TiltI, LauncherConstants.TiltD, this::tuneTiltPID);
		recalibrateTilt();

		m_flywheelLeft.burnFlash();
		m_flywheelRight.burnFlash();
		m_tiltController.burnFlash();
	}

	/**
	 * Sets the target angle for the launcher
	 * @param angle the target angle to move to
	 */
	public void setLauncherAngle(Rotation2d angle) {
		simAngle = angle;
		m_tiltController.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
	}

	public void setLauncherRPM(double speedRPM) {
		simPower = speedRPM;
		m_flywheelLeft.getPIDController().setReference(speedRPM, ControlType.kVelocity);
		m_flywheelRight.getPIDController().setReference(speedRPM, ControlType.kVelocity);
	}

	public double getLauncherRPM() {
		return m_flywheelRight.getEncoder().getVelocity();
	}

	/**
	 * Gets the current angle of the launcher
	 */
	public Rotation2d getCurrentAngle() {
		if (Robot.isSimulation()) {
			return simAngle;
		}
		return Rotation2d.fromDegrees(m_tiltController.getEncoder().getPosition());
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setLauncherPower(double power) {
		simPower = power;
		m_flywheelLeft.set(power);
		m_flywheelRight.set(power);
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the launcher
	 */
	public double getCurrentLauncherPower() {
		if (Robot.isSimulation()) {
			return simPower;
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
	}
}
