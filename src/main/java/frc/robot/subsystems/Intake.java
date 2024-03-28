package frc.robot.subsystems;

import com.chaos131.logging.LogManager;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.IOPorts;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

	TalonFX m_intakeUpper = new TalonFX(CANIdentifiers.IntakeUpper);
	TalonFX m_intakeLower = new TalonFX(CANIdentifiers.IntakeLower);
	private TalonFXConfiguration m_intakeUpperConfig = new TalonFXConfiguration();
	private TalonFXConfiguration m_intakeLowerConfig = new TalonFXConfiguration();
	DigitalInput m_intakeSensor = new DigitalInput(IOPorts.IntakeNoteSensor);

	private double simPower = 0;

	public Intake() {

		m_intakeUpperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		m_intakeUpperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		m_intakeUpperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		m_intakeUpperConfig.CurrentLimits.SupplyCurrentLimit = 60;
		m_intakeUpperConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
		m_intakeUpper.getConfigurator().apply(m_intakeUpperConfig);

		m_intakeLowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		m_intakeLowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		m_intakeLowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		m_intakeLowerConfig.CurrentLimits.SupplyCurrentLimit = 60;
		m_intakeLowerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
		m_intakeLower.getConfigurator().apply(m_intakeLowerConfig);

		var logManager = LogManager.getInstance();
		logManager.addNumber("Intake/UpperCurrentAmps", DebugConstants.IntakeDebugEnable, () -> m_intakeUpper.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber("Intake/LowerCurrentAmps", DebugConstants.IntakeDebugEnable, () -> m_intakeLower.getSupplyCurrent().getValueAsDouble());
		logManager.addNumber("Intake/UpperPowerVoltage", DebugConstants.IntakeDebugEnable, () -> m_intakeUpper.getMotorVoltage().getValueAsDouble());
		logManager.addNumber("Intake/LowerPowerVoltage", DebugConstants.IntakeDebugEnable, () -> m_intakeLower.getMotorVoltage().getValueAsDouble());
		logManager.addBoolean("Intake/HasNote", true, () -> hasNote());
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setIntakePower(double power) {
		simPower = power;
		m_intakeUpper.set(power);
		m_intakeLower.set(power);
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the intake
	 */
	public double getCurrentIntakePower() {
		if(Robot.isSimulation()) {
			return simPower;
		}
		return m_intakeUpper.get();
	}

	public boolean hasNote() {
		return !m_intakeSensor.get();
	}

	@Override
	public void periodic() {
		super.periodic();
		
	}
}
