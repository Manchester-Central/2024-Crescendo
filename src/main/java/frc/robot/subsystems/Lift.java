package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
	private CANSparkFlex m_liftA = new CANSparkFlex(CANIdentifiers.LiftA, MotorType.kBrushless);
	private CANSparkFlex m_liftB = new CANSparkFlex(CANIdentifiers.LiftB, MotorType.kBrushless);
	private PIDTuner m_liftPidTuner;

	private double simHeight = LiftConstants.MinHeightMeters;

	public Lift() {
		m_liftA.getEncoder().setPositionConversionFactor(LiftConstants.LiftEncoderConversionFactor);
		m_liftB.getEncoder().setPositionConversionFactor(LiftConstants.LiftEncoderConversionFactor);
		m_liftPidTuner = new PIDTuner("Lift", Constants.DebugMode, LiftConstants.LiftP, LiftConstants.LiftI, LiftConstants.LiftD, this::tuneLiftPID);
	}

	public void tuneLiftPID(PIDFValue pidValue) {
		m_liftA.getPIDController().setP(pidValue.P);
		m_liftA.getPIDController().setI(pidValue.I);
		m_liftA.getPIDController().setD(pidValue.D);
		m_liftB.getPIDController().setP(pidValue.P);
		m_liftB.getPIDController().setI(pidValue.I);
		m_liftB.getPIDController().setD(pidValue.D);
	}

	/**
	 * Sets the lift to drive to a certain height
	 * @param heightMeters the height to move to
	 */
	public void moveToHeight(double heightMeters) {
		simHeight = heightMeters;

		m_liftA.getPIDController().setReference(heightMeters, ControlType.kPosition);
		m_liftB.getPIDController().setReference(heightMeters, ControlType.kPosition);
	}

	/**
	 * Gets the current height of the lift in meters
	 */
	public double getCurrentHeightMeters() {
		if (Robot.isSimulation()) {
			return simHeight;
		}
		return m_liftA.getEncoder().getPosition();
	}

	@Override
	public void periodic() {
		super.periodic();
		m_liftPidTuner.tune();
	}
}
