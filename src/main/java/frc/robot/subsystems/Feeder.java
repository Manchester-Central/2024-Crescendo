package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.IOPorts;

public class Feeder extends SubsystemBase {
	private DigitalInput m_feederSensorPrimary = new DigitalInput(IOPorts.FeederSensorPrimary);
	private DigitalInput m_feederSensorSecondary = new DigitalInput(IOPorts.FeederSensorSecondary);
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
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.b().getAsBoolean(); //x on keyboard 0
		}
		return m_feederSensorPrimary.get();
	}
	
	public void grabAndHoldPiece(double grabSpeed) {
		if (hasNoteAtSecondary()) {
			setFeederPower(-0.1);
		} else if(hasNoteAtPrimary()) {
			setFeederPower(0);
		} else {
			setFeederPower(grabSpeed);
		}
	}

	public boolean hasNoteAtSecondary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.a().getAsBoolean(); //z on keyboard 0
		}
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
