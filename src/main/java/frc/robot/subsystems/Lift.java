package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.IOPorts;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
	private DigitalInput m_bottomSensor = new DigitalInput(IOPorts.LiftBottomSensor);
	private TalonFX m_liftA = new TalonFX(CANIdentifiers.LiftA);
	private TalonFX m_liftB = new TalonFX(CANIdentifiers.LiftB);
	private PIDTuner m_liftPidTuner;
	private TalonFXConfiguration m_liftAConfig = new TalonFXConfiguration();
	private TalonFXConfiguration m_liftBConfig = new TalonFXConfiguration();
	private PositionVoltage m_positionVoltage = new PositionVoltage(0);
	
	private double simHeight = LiftConstants.MinHeightMeters;

	private boolean m_hasSeenBottom = false;

	public Lift() {

		m_liftAConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftAConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		m_liftAConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftAConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftA.getConfigurator().apply(m_liftAConfig);
		
		m_liftBConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftBConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		m_liftBConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftBConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftB.getConfigurator().apply(m_liftBConfig);

		m_liftPidTuner = new PIDTuner("Lift", Constants.DebugMode, LiftConstants.LiftP, LiftConstants.LiftI,
				LiftConstants.LiftD, this::tuneLiftPID);
	}

	public void setSpeed(double speed){
		m_liftA.set(speed);
		m_liftB.set(speed);
	}

	public void tuneLiftPID(PIDFValue pidValue) {
		var slot0 = new Slot0Configs();
		slot0.kP = pidValue.P;
		slot0.kI = pidValue.I;
		slot0.kD = pidValue.D;
		m_liftAConfig.Slot0 = slot0;
		m_liftBConfig.Slot0 = slot0;
		m_liftA.getConfigurator().apply(m_liftAConfig.Slot0);
		m_liftB.getConfigurator().apply(m_liftBConfig.Slot0);
	}

	/**
	 * Sets the lift to drive to a certain height
	 * @param heightMeters the height to move to
	 */
	public void moveToHeight(double heightMeters) {
		simHeight = heightMeters;
		m_positionVoltage.Slot = 0;
		m_liftA.setControl(m_positionVoltage.withPosition(heightMeters));
		m_liftB.setControl(m_positionVoltage.withPosition(heightMeters));
	}

	/**
	 * Gets the current height of the lift in meters
	 */
	public double getCurrentHeightMeters() {
		if (Robot.isSimulation()) {
			return simHeight;
		}
		return m_liftA.getPosition().getValueAsDouble();
	}

	public boolean atBottom() {
		return m_bottomSensor.get();
	}
	
	public boolean hasSeenBottom(){
		return m_hasSeenBottom;
	}

	@Override
	public void periodic() {
		super.periodic();
		m_liftPidTuner.tune();
		if(atBottom() && !hasSeenBottom()){
			m_hasSeenBottom = true;
			m_liftA.setPosition(LiftConstants.MinHeightMeters);
			m_liftB.setPosition(LiftConstants.MinHeightMeters);
		}
	}
}
