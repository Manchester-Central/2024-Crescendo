package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.IOPorts;
import frc.robot.Constants.LiftConstants;
import frc.robot.Robot;

public class Lift extends SubsystemBase {
	private DigitalInput m_bottomSensor = new DigitalInput(IOPorts.LiftBottomSensor);
	private TalonFX m_liftLeft = new TalonFX(CANIdentifiers.LiftLeft);
	private TalonFX m_liftRight = new TalonFX(CANIdentifiers.LiftRight);
	private PIDTuner m_liftPidTuner;
	private TalonFXConfiguration m_liftLeftConfig = new TalonFXConfiguration();
	private TalonFXConfiguration m_liftRightConfig = new TalonFXConfiguration();
	private PositionVoltage m_positionVoltage = new PositionVoltage(0);
	
	private double m_simHeight = LiftConstants.MinHeightMeters + 0.5; // Start off higher so we can see the lift move down
	private double m_simPower = 0;
	private double m_simMaxMetersChangePerLoop = 0.1;
	private PIDController m_simPid = new PIDController(1, 0, 0);
	

	private boolean m_hasSeenBottom = false;
	private double m_targetHeightMeters = 0; 

	public Lift() {

		m_liftLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		m_liftLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftLeftConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftLeft.getConfigurator().apply(m_liftLeftConfig);
		
		m_liftRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		m_liftRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftRightConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftRight.getConfigurator().apply(m_liftRightConfig);

		m_liftPidTuner = new PIDTuner("Lift", Constants.DebugMode, LiftConstants.LiftP, LiftConstants.LiftI,
				LiftConstants.LiftD, this::tuneLiftPID);
	}

	public void setSpeed(double speed) {
		if (!hasSeenBottom()){
			speed = MathUtil.clamp(speed, -LiftConstants.MaxSpeedBeforeBottom, 0);
		}
		else if (getCurrentHeightMeters() >= LiftConstants.MaxHeightMeters){
			speed = MathUtil.clamp(speed, -LiftConstants.MaxSpeed, 0);
		}
		else if (getCurrentHeightMeters() <= LiftConstants.MinHeightMeters){
			speed = MathUtil.clamp(speed, 0, LiftConstants.MaxSpeed);
		} 
		
		if (Robot.isSimulation()) {
			m_simPower = speed;
		}

		m_liftLeft.set(speed);
		m_liftRight.set(speed);
	}

	public boolean atTargetHeight(double targetHeight) {
		return Math.abs(getCurrentHeightMeters() - targetHeight) <= LiftConstants.LiftToleranceMeters;
	}

	public void tuneLiftPID(PIDFValue pidValue) {
		var slot0 = new Slot0Configs();
		slot0.kP = pidValue.P;
		slot0.kI = pidValue.I;
		slot0.kD = pidValue.D;
		m_liftLeftConfig.Slot0 = slot0;
		m_liftRightConfig.Slot0 = slot0;
		m_liftLeft.getConfigurator().apply(m_liftLeftConfig.Slot0);
		m_liftRight.getConfigurator().apply(m_liftRightConfig.Slot0);
	}

	/**
	 * Sets the lift to drive to a certain height
	 * @param heightMeters the height to move to
	 */
	public void moveToHeight(double heightMeters) {
		m_targetHeightMeters = heightMeters;
		if (!hasSeenBottom()) {
			m_simPower = 0;
			// If we haven't seen the bottom, don't allow Closed-loop control
			return;
		}

		heightMeters = MathUtil.clamp(heightMeters, LiftConstants.MinHeightMeters, LiftConstants.MaxHeightMeters);

		if (Robot.isSimulation()) {
			m_simPower = MathUtil.clamp(m_simPid.calculate(m_simHeight, heightMeters), -1.0, 1.0);
		}
		m_positionVoltage.Slot = 0;
		m_liftLeft.setControl(m_positionVoltage.withPosition(heightMeters));
		m_liftRight.setControl(m_positionVoltage.withPosition(heightMeters));
	}

	/**
	 * Gets the current height of the lift in meters
	 */
	public double getCurrentHeightMeters() {
		if (Robot.isSimulation()) {
			return m_simHeight;
		}
		return m_liftLeft.getPosition().getValueAsDouble();
	}

	public boolean atBottom() {
		if (Robot.isSimulation()) {
			return m_simHeight < LiftConstants.MinHeightMeters;
		}
		return !m_bottomSensor.get();
	}
	
	public boolean hasSeenBottom(){
		return m_hasSeenBottom;
	}

	@Override
	public void periodic() {
		super.periodic();
		m_liftPidTuner.tune();
		if (atBottom() && !hasSeenBottom()){
			m_hasSeenBottom = true;
			m_liftLeft.setPosition(LiftConstants.MinHeightMeters + 0.05);
			m_liftRight.setPosition(LiftConstants.MinHeightMeters + 0.05);
		}

		if (Robot.isSimulation()) {
			m_simHeight += m_simPower * m_simMaxMetersChangePerLoop;
		}

		SmartDashboard.putBoolean("Lift/AtBottom", atBottom());
		SmartDashboard.putBoolean("Lift/SeenBottom", hasSeenBottom());

		SmartDashboard.putNumber("Lift/CurrentHeightMeters", getCurrentHeightMeters());
		SmartDashboard.putNumber("Lift/LeftHeight", m_liftLeft.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Lift/RightHeight", m_liftRight.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Lift/TargetHeightMeters", m_targetHeightMeters);
	}
}
