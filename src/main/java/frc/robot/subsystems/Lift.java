package frc.robot.subsystems;

import java.util.Optional;

import com.chaos131.logging.LogManager;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.DebugConstants;
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
	
	private double m_simHeight = LiftConstants.MinHeightMeters; // Start off higher so we can see the lift move down
	private double m_simPower = 0;
	private double m_simMaxMetersChangePerLoop = 0.1;
	private PIDController m_simPid = new PIDController(1, 0, 0);
	

	private boolean m_hasSeenBottom = false;
	private Optional<Double> m_targetHeightMeters = Optional.empty(); 

	public static boolean SafeftyLimtEnabled = true;

	public Lift() {

		m_liftLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		m_liftLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		m_liftLeftConfig.CurrentLimits.SupplyCurrentLimit = 40;
		m_liftLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftLeftConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = LiftConstants.LiftRampRate;
		m_liftLeft.getConfigurator().apply(m_liftLeftConfig);
		
		m_liftRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_liftRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		m_liftRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		m_liftRightConfig.CurrentLimits.SupplyCurrentLimit = 40;
		m_liftRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		m_liftRightConfig.Feedback.SensorToMechanismRatio = LiftConstants.LiftEncoderConversionFactor;
		m_liftRightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = LiftConstants.LiftRampRate;
		m_liftRight.getConfigurator().apply(m_liftRightConfig);

		m_liftPidTuner = new PIDTuner("Lift", DebugConstants.LiftDebugEnable, LiftConstants.LiftP, LiftConstants.LiftI,
				LiftConstants.LiftD, this::tuneLiftPID);

		var logManager = LogManager.getInstance();

		logManager.addBoolean("Lift/AtBottom", true, () -> atBottom());
		logManager.addBoolean("Lift/SeenBottom", DebugConstants.LiftDebugEnable, () -> hasSeenBottom());

		logManager.addNumber("Lift/CurrentHeightMeters", true, () -> getCurrentHeightMeters());
		logManager.addNumber("Lift/LeftHeightMeters", DebugConstants.LiftDebugEnable, () -> m_liftLeft.getPosition().getValueAsDouble());
		logManager.addNumber("Lift/RightHeightMeters", DebugConstants.LiftDebugEnable, () -> m_liftRight.getPosition().getValueAsDouble());
		logManager.addNumber("Lift/TargetHeightMeters", DebugConstants.LiftDebugEnable, () -> m_targetHeightMeters.orElse(-1.0));
		logManager.addNumber("Lift/LeftPowerVoltage", DebugConstants.LiftDebugEnable, () -> m_liftLeft.getMotorVoltage().getValueAsDouble());
		logManager.addNumber("Lift/RightPowerVoltage", DebugConstants.LiftDebugEnable, () -> m_liftRight.getMotorVoltage().getValueAsDouble());
		logManager.addNumber("Lift/LeftCurrentAmps", DebugConstants.LiftDebugEnable, () -> m_liftLeft.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber("Lift/RightCurrentAmps", DebugConstants.LiftDebugEnable, () -> m_liftRight.getSupplyCurrent().getValueAsDouble());
	}

	public void setSpeed(double speed) {

		if(SafeftyLimtEnabled){
			if (!hasSeenBottom()){
				speed = MathUtil.clamp(speed, -LiftConstants.MaxSpeedBeforeBottom, 0);
			}
			else if (getCurrentHeightMeters() >= LiftConstants.MaxHeightMeters){
				speed = MathUtil.clamp(speed, -LiftConstants.MaxSpeed, 0);
			}
			else if (getCurrentHeightMeters() <= LiftConstants.MinHeightMeters){
				speed = MathUtil.clamp(speed, 0, LiftConstants.MaxSpeed);
			} 
		}
		
		if (Robot.isSimulation()) {
			m_simPower = speed;
		}

		m_liftLeft.set(speed);
		m_liftRight.set(speed);

		m_targetHeightMeters = Optional.empty();
	}

	public boolean atTargetHeight(double targetHeight) {
		return Math.abs(getCurrentHeightMeters() - targetHeight) <= LiftConstants.LiftToleranceMeters;
	}

	public void tuneLiftPID(PIDFValue pidValue) {
		var slot0 = new Slot0Configs();
		slot0.kP = pidValue.P;
		slot0.kI = pidValue.I;
		slot0.kD = pidValue.D;
		slot0.kG = LiftConstants.LiftG;
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
		m_targetHeightMeters = Optional.of(heightMeters);
		if (!hasSeenBottom()) {
			m_simPower = 0;
			// If we haven't seen the bottom, don't allow Closed-loop control
			return;
		}

		heightMeters = MathUtil.clamp(heightMeters, LiftConstants.MinHeightMeters, LiftConstants.MaxHeightMeters);
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
		if (Robot.isSimulation()) {
			return true;
		}
		return m_hasSeenBottom;
	}

	@Override
	public void periodic() {
		super.periodic();
		m_liftPidTuner.tune();
		if (atBottom() && !hasSeenBottom()){
			m_hasSeenBottom = true;
			m_liftLeft.setPosition(0);
			m_liftRight.setPosition(0);
		}

		if (Robot.isSimulation()) {

			if (m_targetHeightMeters.isPresent()) {
				m_simPower = MathUtil.clamp(m_simPid.calculate(m_simHeight, m_targetHeightMeters.get()), -1.0, 1.0);
			}
			m_simHeight += m_simPower * m_simMaxMetersChangePerLoop;
		}

	}

	public void changeNeutralMode(NeutralModeValue neutralMode) {
		var neutralModeConfigLeft = m_liftLeftConfig.MotorOutput;
		var neutralModeConfigRight = m_liftRightConfig.MotorOutput;
		neutralModeConfigLeft.NeutralMode = neutralMode;
		neutralModeConfigRight.NeutralMode = neutralMode;
		m_liftLeft.getConfigurator().apply(neutralModeConfigLeft);
		m_liftRight.getConfigurator().apply(neutralModeConfigRight);
	}
}
