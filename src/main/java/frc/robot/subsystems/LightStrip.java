package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip extends SubsystemBase{
    private AddressableLED m_leds;  
    private AddressableLEDBuffer m_buffer;
    private final int NumLeds = 15;
    // private Object m_mutex = new Object();
    private Supplier<Boolean> m_intakeSupplier;
    private Supplier<Boolean> m_feederSupplier;

    public enum Color {
        RED (255, 0, 0),
        GREEN(0, 255, 0),
        BLUE (0, 0, 255),
        CHAOS_ORANGE (255, 30, 0),
        OFF (0, 0, 0),
        YELLOW (255, 255, 0),
        WHITE(255, 255, 255);
        
        int red;
        int green;
        int blue;

        Color(int r, int g, int b) {
            red = r;
            green = g;
            blue = b;
        }
    }

    public LightStrip(Supplier<Boolean> intakeHasNote, Supplier<Boolean> feederHasNote) {
        m_intakeSupplier = intakeHasNote;
        m_feederSupplier = feederHasNote;
        m_leds = new AddressableLED(0);
        m_leds.setLength(NumLeds);
        m_buffer = new AddressableLEDBuffer(NumLeds);
        m_leds.start();
    }

    @Override
    public void periodic() {
        // Set color to orange if the robot is not connected to the driver station
        if(DriverStation.isDSAttached() == false) {
            setSingleColor(Color.CHAOS_ORANGE);
            return;
        }

        // If the robot is disabled, display the current alliance if there is one
        if(DriverStation.isDisabled()) {
            if(DriverStation.getAlliance().isPresent()) {
                if(DriverStation.getAlliance().get() == Alliance.Red) {
                    setSingleColor(Color.RED);
                } else {
                    setSingleColor(Color.BLUE);
                }
            }
        } else {
            // Logic for an enabled robot
            if(m_intakeSupplier.get() == true) {
                setSingleColor(Color.YELLOW);
            } else if(m_feederSupplier.get() == true && m_intakeSupplier.get() == false) {
                setSingleColor(Color.GREEN);
            } else {
                setSingleColor(Color.WHITE);
            }
        }
    }

    public void setSingleColor(int red, int green, int blue) {
        for(int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, red, green, blue);
        }

        m_leds.setData(m_buffer);
    }
    
    public void setSingleColor(Color color) {
        setSingleColor(color.red, color.green, color.blue);
    }
}