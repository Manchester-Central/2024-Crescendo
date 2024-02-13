package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.IOPorts;

public class Feeder extends SubsystemBase {
	public DigitalInput m_feederSensorPrimary = new DigitalInput(IOPorts.FeederSensorPrimary);
	public DigitalInput m_feederSensorSecondary = new DigitalInput(IOPorts.FeederSensorSecondary);
	CANSparkFlex m_feederMotor = new CANSparkFlex(CANIdentifiers.Feeder, MotorType.kBrushless);

	private double simPower = 0;

	public Feeder() {
		//
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setFeederPower(double power) {
		simPower = power;
		m_feederMotor.set(power);
	}

	public boolean hasNoteAtPrimary(){
		return m_feederSensorPrimary.get();
	}
	
	public boolean hasNoteAtSecondary(){
		return m_feederSensorSecondary.get();
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the feeder
	 */
	public double getCurrentFeederPower() {
		if (Robot.isSimulation()) {
			return simPower;
		}
		return m_feederMotor.get();
	}
}