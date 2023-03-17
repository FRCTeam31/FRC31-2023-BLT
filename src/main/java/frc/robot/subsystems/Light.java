package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {
    AddressableLED mFrontGreenStrip;

    public Light() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        mFrontGreenStrip = new AddressableLED(8);
    
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(1);
        mFrontGreenStrip.setLength(m_ledBuffer.getLength());
    
        // Set the data
        mFrontGreenStrip.setData(m_ledBuffer);
        mFrontGreenStrip.start();
         
        setFrontStripColor(0, 225, 0);
    }

    public void setFrontStripColor(int red, int green, int blue) {
        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(24);
    
        // Set the data
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 225, 0);
        }

        mFrontGreenStrip.setData(m_ledBuffer);
    }
}


