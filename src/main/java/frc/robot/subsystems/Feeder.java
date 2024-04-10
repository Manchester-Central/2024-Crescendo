package frc.robot.subsystems;

import com.chaos131.logging.LogManager;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.DebugConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
	private SparkLimitSwitch m_feederSensorPrimary;
	private SparkLimitSwitch m_feederSensorSecondary;
	private SparkLimitSwitch m_feederSensorTertiary;
	private Debouncer m_primaryDebouncer = new Debouncer(0.05, DebounceType.kBoth);
	private Debouncer m_secondaryDebouncer = new Debouncer(0.05, DebounceType.kBoth);
	private boolean m_hasNoteAtPrimary = false;
	private boolean m_hasNoteAtSecondary = false;

	CANSparkFlex m_feederMainMotor = new CANSparkFlex(CANIdentifiers.FeederMain, MotorType.kBrushless);
	CANSparkFlex m_feederTrapMotor = new CANSparkFlex(CANIdentifiers.FeederTrap, MotorType.kBrushless);

	private double simPower = 0;

	public Feeder() {
		m_feederMainMotor.restoreFactoryDefaults();
		m_feederTrapMotor.restoreFactoryDefaults();
		m_feederSensorPrimary = m_feederMainMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_feederSensorSecondary = m_feederMainMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_feederSensorTertiary = m_feederTrapMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_feederSensorPrimary.enableLimitSwitch(false);
		m_feederSensorSecondary.enableLimitSwitch(false);
		m_feederSensorTertiary.enableLimitSwitch(false);	
		m_feederTrapMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);

		m_feederMainMotor.setIdleMode(IdleMode.kBrake);
		m_feederTrapMotor.setIdleMode(IdleMode.kBrake);
		
		m_feederMainMotor.setSmartCurrentLimit(80);
		m_feederTrapMotor.setSmartCurrentLimit(80);

		m_feederMainMotor.setInverted(true);
		m_feederTrapMotor.setInverted(true);
		m_feederMainMotor.setOpenLoopRampRate(0.1);
		m_feederTrapMotor.setOpenLoopRampRate(0.1);

		m_feederMainMotor.burnFlash();
		m_feederTrapMotor.burnFlash();

		var logManager = LogManager.getInstance();
		logManager.addBoolean("Feeder/HasNoteAtPrimary", true, () -> hasNoteAtPrimary());
		logManager.addBoolean("Feeder/HasNoteAtSecondary", true, () -> hasNoteAtSecondary());
		logManager.addBoolean("Feeder/HasNoteAtTertiary", true, () -> hasNoteAtTertiary());

		logManager.addNumber("FeederMain/MotorPosition", DebugConstants.FeederDebugEnable, () -> m_feederMainMotor.getEncoder().getPosition());
		logManager.addNumber("FeederMain/CurrentAmps", DebugConstants.FeederDebugEnable, () -> m_feederMainMotor.getOutputCurrent());
		logManager.addNumber("FeederMain/AppliedOutput", DebugConstants.FeederDebugEnable, () -> m_feederMainMotor.getAppliedOutput());

		logManager.addNumber("FeederTrap/CurrentAmps", DebugConstants.FeederDebugEnable, () -> m_feederTrapMotor.getOutputCurrent());
		logManager.addNumber("FeederTrap/AppliedOutput", DebugConstants.FeederDebugEnable, () -> m_feederTrapMotor.getAppliedOutput());
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setFeederPower(double power) {
		setFeederPower(power, power);
	}

	public void setFeederPower(double mainPower, double trapPower) {
		simPower = mainPower;
		m_feederMainMotor.set(mainPower);
		m_feederTrapMotor.set(trapPower);
	}
	
	public void grabAndHoldPiece(double grabSpeed) {
		if (hasNoteAtTertiary()) {
			setFeederPower(-0.05);
		} else if(hasNoteAtSecondary() || hasNoteAtPrimary()) {
			setFeederPower(0.05);
		} else {
			setFeederPower(grabSpeed);
		}
	}

	public boolean hasNoteAtPrimary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.x().getAsBoolean(); //c on keyboard 0
		}
		return m_hasNoteAtPrimary;
	}

	public boolean hasNoteAtSecondary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.b().getAsBoolean(); //x on keyboard 0
		}
		return m_hasNoteAtSecondary;
	}

	public boolean hasNoteAtTertiary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.a().getAsBoolean(); //z on keyboard 0
		}
		return m_feederSensorTertiary.isPressed();
	}

	/**
	 * Checks is there is a note at either sensor
	 */
	public boolean hasNote() {
		return hasNoteAtPrimary() || hasNoteAtSecondary() || hasNoteAtTertiary();
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the feeder
	 */
	public double getCurrentFeederPower() {
		if (Robot.isSimulation()) {
			return simPower;
		}
		return m_feederMainMotor.get();
	}
	
	@Override
	public void periodic() {
		super.periodic();
		m_hasNoteAtPrimary = m_primaryDebouncer.calculate(m_feederSensorPrimary.isPressed());
		m_hasNoteAtSecondary = m_secondaryDebouncer.calculate(m_feederSensorSecondary.isPressed());
	}
}
