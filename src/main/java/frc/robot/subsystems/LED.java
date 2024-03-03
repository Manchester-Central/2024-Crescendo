package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED m_strip;
    private AddressableLEDBuffer m_buffer;
    private Object m_mutex = new Object();

    @Override
    public void periodic() {
        m_buffer = new AddressableLEDBuffer(0); // Set length when we know how many leds will be in the strip
        m_strip = new AddressableLED(0); // Add a constants value to Constants.java when we have a physical PWM port number
        m_strip.setLength(m_buffer.getLength());
        m_strip.start();
    }

    public void setRGB(int red, int green, int blue) {
        synchronized(m_mutex) {
            for(int i = 0; i < m_buffer.getLength()) {
                m_buffer.setRGB(i, red, green, blue);
            }
            m_strip.setData(m_buffer);
        }
    }
    
}
