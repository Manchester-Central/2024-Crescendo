package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
	public Launcher() {
		
	}
	/** 
	 * Rotate the motor with a target rotation per minute
	 * @param rpm Rotations per minute
	*/
	public void launchWithTargetRPM(double rpm){

	}
	/**
	 * Rotate the motor with percentage of power
	 * @param speed 0 to 1
	 */
	public void launchWithPercentSpeed(double speed){

	}
	/**
	 * Rotate launcher to angle
	 * @param tiltAngle angle in degrees
	 */
	public void setTiltAngle(Rotation2d tiltAngle){

	}

	/**
	 * stop
	 */
	public void stop(){

	}
	
}
