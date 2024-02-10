package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Launcher extends SubsystemBase {

	private Rotation2d simAngle = Rotation2d.fromDegrees(0);
	private double simPower = 0;

	public Launcher() {
		//
	}

	/**
	 * Sets the target angle for the launcher
	 * @param angle the target angle to move to
	 */
	public void setLauncherAngle(Rotation2d angle) {
		simAngle = angle;
	}

	/**
	 * Gets the current angle of the launcher
	 */
	public Rotation2d getCurrentAngle() {
		if (Robot.isSimulation()) {
			return simAngle;
		}
		// TODO: update with real value
		return Rotation2d.fromDegrees(0);
	}

	/**
	 * Sets the run power
	 * @param power the duty cycle [-1, 1] power to run at
	 */
	public void setLauncherPower(double power) {
		simPower = power;
	}

	/** 
	 * Gets the current duty cycle power [-1, 1] of the launcher
	 */
	public double getCurrentLauncherPower() {
		if (Robot.isSimulation()) {
			return simPower;
		}
		// TODO: update with real value
		return 0;
	}
}
