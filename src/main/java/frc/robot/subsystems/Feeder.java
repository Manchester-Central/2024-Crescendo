package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
	private SparkLimitSwitch m_feederSensorPrimary;
	private SparkLimitSwitch m_feederSensorSecondary;
	CANSparkFlex m_feederMotor = new CANSparkFlex(CANIdentifiers.Feeder, MotorType.kBrushless);

	private double simPower = 0;

	public Feeder() {
		m_feederSensorPrimary = m_feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_feederSensorSecondary = m_feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_feederSensorPrimary.enableLimitSwitch(false);
		m_feederSensorSecondary.enableLimitSwitch(false);
		m_feederMotor.setInverted(true);
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setFeederPower(double power) {
		simPower = power;
		m_feederMotor.set(power);
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

	public boolean hasNoteAtPrimary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.b().getAsBoolean(); //x on keyboard 0
		}
		return m_feederSensorPrimary.isPressed();
	}

	public boolean hasNoteAtSecondary(){
		if (Robot.isSimulation()) {
			return RobotContainer.SimKeyboard.a().getAsBoolean(); //z on keyboard 0
		}
		return m_feederSensorSecondary.isPressed();
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
	
	@Override
	public void periodic() {
		super.periodic();
		SmartDashboard.putBoolean("Feeder/HasNoteAtPrimary", hasNoteAtPrimary());
		SmartDashboard.putBoolean("Feeder/HasNoteAtSecondary", hasNoteAtSecondary());

		SmartDashboard.putNumber("Feeder/MotorPosition", m_feederMotor.getEncoder().getPosition());
	}
}
