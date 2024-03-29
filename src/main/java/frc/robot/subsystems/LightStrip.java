package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip extends SubsystemBase{
    private AddressableLED m_leds;  
    private AddressableLEDBuffer m_buffer;
    private final int NumLeds = 0;
    private Object m_mutex = new Object();
    private Supplier<Boolean> m_intakeSupplier;
    private Supplier<Boolean> m_feederSupplier;

    public LightStrip(Supplier<Boolean> intakeHasNote, Supplier<Boolean> feederHasNote) {
        m_intakeSupplier = intakeHasNote;
        m_feederSupplier = feederHasNote;
        m_leds.setLength(NumLeds);
        m_buffer = new AddressableLEDBuffer(NumLeds);
    }

    public void setSingleColor(int red, int green, int blue) {
        for(int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, red, green, blue);
        }

        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void periodic() {

        if(DriverStation.isDSAttached() == false) {
            setSingleColor(255, 30, 0);
            return;
        }

        // If the robot is disabled, display the current alliance if there is one
        if(DriverStation.isDisabled()) {
            if(DriverStation.getAlliance().isPresent()) {
                if(DriverStation.getAlliance().get() == Alliance.Red) {
                    setSingleColor(255, 0, 0); // Set all LEDs to red if we are disabled on red
                } else {
                    setSingleColor(0, 0, 255);
                }
            }
        } else {
            // Logic for an enabled robot
            if(m_intakeSupplier.get() == true) {
                setSingleColor(255, 255, 0); // Yellow
            } else if(m_feederSupplier.get() == true && m_intakeSupplier.get() == false) {
                setSingleColor(0, 255, 0); // Green
            } else {
                setSingleColor(255, 255, 255); // White
            }
        }
    }
}
