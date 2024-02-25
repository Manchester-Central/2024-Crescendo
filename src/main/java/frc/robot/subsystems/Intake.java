package frc.robot.subsystems;

import com.chaos131.logging.LogManager;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;

public class Intake extends SubsystemBase {

	CANSparkMax m_intakeUpper = new CANSparkMax(CANIdentifiers.IntakeUpper, MotorType.kBrushless);
	CANSparkMax m_intakeLower = new CANSparkMax(CANIdentifiers.IntakeLower, MotorType.kBrushless);

	private double simPower = 0;

	public Intake() {
		m_intakeUpper.restoreFactoryDefaults();
		m_intakeLower.restoreFactoryDefaults();
		m_intakeUpper.setIdleMode(IdleMode.kCoast);
		m_intakeLower.setIdleMode(IdleMode.kCoast);
		m_intakeUpper.setOpenLoopRampRate(0.1);
		m_intakeLower.setOpenLoopRampRate(0.1);
		m_intakeUpper.burnFlash();
		m_intakeLower.burnFlash();

		var logManager = LogManager.getInstance();
		logManager.addNumber("Intake/UpperCurrentAmps", Constants.DebugMode, () -> m_intakeUpper.getOutputCurrent());
		logManager.addNumber("Intake/LowerCurrentAmps", Constants.DebugMode, () -> m_intakeLower.getOutputCurrent());
		logManager.addNumber("Intake/UpperAppliedOutput", Constants.DebugMode, () -> m_intakeUpper.getAppliedOutput());
		logManager.addNumber("Intake/LowerAppliedOutput", Constants.DebugMode, () -> m_intakeLower.getAppliedOutput());

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

	@Override
	public void periodic() {
		super.periodic();
		
	}
}
