package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gondola extends SubsystemBase {
	public Gondola() {
		
	}

	/**Moves the Gondola */
	public void elevateManual(double speed){

	}

	
	/**
	 * Moves the Gondola to specific postion
	 * @param heightMeters the target point in meters
	 */
	public void elevateToPostion(double heightMeters){

	}

	/**returns if the Gondola has a note */
	public boolean hasNote(){
		return false;
	}

	/**runs the Gondola's feeder*/
	public void setFeedSpeed(double speed){

	}

	/**Stop Gondola's feeder*/
	public void stopFeeder(){

	}

	/**top the Gondola's Elavator*/
	public void stopElavator(){

	}

	/**Stop the Gondola*/
	public void stopAll(){
		stopFeeder();
		stopElavator();
	}

}
