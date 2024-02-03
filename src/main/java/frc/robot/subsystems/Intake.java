package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	
	CANSparkMax m_intakeUpper = new CANSparkMax(42,MotorType.kBrushless);
	CANSparkMax m_intakeLower = new CANSparkMax(43,MotorType.kBrushless);
	public Intake() {
		//m_motor = new TalonFX(0);
		//m_config = new TalonFXConfiguration();

		//m_motor.getConfigurator().apply(m_config);
	}
	public void runSpeed(double speed){
		m_intakeUpper.set(speed);
		m_intakeLower.set(speed);
	}
}
