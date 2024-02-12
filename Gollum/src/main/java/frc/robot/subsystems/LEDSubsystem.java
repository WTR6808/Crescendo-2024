package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem m_instance = null;

    // LEDS
    private static AddressableLED m_leds = null;
    // Buffer for LED data, change this to change LED colors
    private static AddressableLEDBuffer m_ledBuffer = null;

    // First color to be displayed on rainbow start
    int m_rainbowFirstPixelHue = 0;

    public static LEDSubsystem Instance() {
        if (m_instance == null) {
            m_instance = new LEDSubsystem();
        } 
        return m_instance;
    }

    private void LEDSubsystem() {
        // Create LEDS class
        m_leds = new AddressableLED(Constants.LED_Constants.LED_ROBORIO_PWM_HEADER);
        // Create buffer for LEDS
        m_ledBuffer = new AddressableLEDBuffer(Constants.LED_Constants.LED_LENGTH);
        // Set the buffer
        m_leds.setData(m_ledBuffer);
        // Turn on the LEDS
        m_leds.start();
    }

    public void setAllColors(int red, int green, int blue) {
        Color m_color = new Color(red, green, blue);
        for (int i = 0; i < m_ledBuffer.getLength(); i ++) {
            m_ledBuffer.setLED(i, m_color);
        }
    }

    public void setAllColors(Color m_color) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, m_color);
        }
    }

    public void setIndividualColor(int pos, int red, int green, int blue) {
        Color m_color = new Color(red, green, blue);
        m_ledBuffer.setLED(pos, m_color);
    }
  
    public void setIndividualColor(int pos, Color m_color) {
        m_ledBuffer.setLED(pos, m_color);
    }

    public void setSectionColor(int pos1, int pos2, int red, int green, int blue) {
        Color m_color = new Color(red, green, blue);
        for (int i = pos1; i < pos2; i++) {
            m_ledBuffer.setLED(i, m_color);
        }
    }
    
    public void setSectionColor(int pos1, int pos2, Color m_color) {
        for (int i = pos1; i < pos2; i++) {
            m_ledBuffer.setLED(i, m_color);
        }
    }

    public void updateColors() {
        m_leds.setData(m_ledBuffer);
    }
    
    // Directly stolen from web tutorial page :P
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
}
