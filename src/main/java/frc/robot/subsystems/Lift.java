package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANIdentifiers;

public class Lift extends SubsystemBase {
	private CANSparkFlex m_liftA = new CANSparkFlex(CANIdentifiers.LiftA, MotorType.kBrushless);
	private CANSparkFlex m_liftB = new CANSparkFlex(CANIdentifiers.LiftB, MotorType.kBrushless);
	// TODO: move to constants
	public static final double MinHeightMeters = 0.35;
	public static final double DefaultLaunchMeters = 0.6;
	public static final double DefaultAmpMeters = 1.0;

	private double simHeight = MinHeightMeters;

	public Lift() {
		//
	}

	/**
	 * Sets the lift to drive to a certain height
	 * @param heightMeters the height to move to
	 */
	public void moveToHeight(double heightMeters) {
		simHeight = heightMeters;
		// TODO: tell the motor to move to the height in closed loop form
	}

	/**
	 * Gets the current height of the lift in meters
	 */
	public double getCurrentHeightMeters() {
		if (Robot.isSimulation()) {
			return simHeight;
		}
		// TODO: get real value
		return 0;
	}
}
