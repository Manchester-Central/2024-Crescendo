package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;

public class Intake extends SubsystemBase {

	CANSparkMax m_intakeUpper = new CANSparkMax(CANIdentifiers.IntakeUpper, MotorType.kBrushless);
	CANSparkMax m_intakeLower = new CANSparkMax(CANIdentifiers.IntakeLower, MotorType.kBrushless);

	private double simPower = 0;

	public Intake() {

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
}
