package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	protected TalonFX m_motor;
	protected TalonFXConfiguration m_config;
	
	public Intake() {
		m_motor = new TalonFX(0);
		m_config = new TalonFXConfiguration();

		m_motor.getConfigurator().apply(m_config);
	}

	public void intake(){

	}

	public void spit(){

	}

	public void stop(){
		
	}
}
