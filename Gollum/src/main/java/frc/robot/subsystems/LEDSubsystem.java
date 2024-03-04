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

    private int[][] m_ledSections = {
        {1, 17}, // Front Left
        {20, 32}, // Front Top
        {33, 48}, // Front Right
        {49, 60}, // Front Bottom
        {61, 73}, // Front Side Left, I know this one and the next are both 14
        {74, 89} // Front Side Right
    };
    private Color[] m_ledStripe = { 
        new Color(0, 255, 0),
        new Color(0, 255, 0),
        new Color(0, 255, 0),
    };
    private int m_stripeTime = 0;
    private int m_currentStripeTime = 0;
    private int m_stripeStart = 0;
    private boolean m_isOff = false;
    // First color to be displayed on rainbow start
    int m_rainbowFirstPixelHue = 0;

    public static LEDSubsystem Instance() {
        if (m_instance == null) {
            m_instance = new LEDSubsystem();
        } 
        return m_instance;
    }

    private LEDSubsystem() {
        // Create LEDS class
        m_leds = new AddressableLED(Constants.LED_Constants.LED_ROBORIO_PWM_HEADER);
        // Set length to LEDS
        m_leds.setLength(Constants.LED_Constants.LED_LENGTH);
        // Create buffer for LEDS
        m_ledBuffer = new AddressableLEDBuffer(Constants.LED_Constants.LED_LENGTH);

        // Turn on color IE. Purple
        setAllColors(255, 0, 255);

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

    public void stripe(int[] stripes, int space, boolean[] reverse, int updateAtThisManyCycles) {
        if (updateAtThisManyCycles != m_stripeTime) m_stripeTime = updateAtThisManyCycles;
        if (m_currentStripeTime == m_stripeTime) {
            int stripe, start, end, patchStart;
            for (int n = 0; n < stripes.length; n++) {
                stripe = stripes[n];
                start = m_ledSections[stripe][0];
                end = m_ledSections[stripe][1];
                if (!reverse[n])
                    for (int currentLed = start + m_stripeStart; currentLed < end; currentLed += space){
                        patchStart = currentLed;
                        for(int i = 0; (m_ledStripe.length != 0) && (currentLed < patchStart + m_ledStripe.length) && (currentLed <= end); currentLed++) {
                            setIndividualColor(currentLed, m_ledStripe[i]);
                            i++;
                        }
                    }
                else {
                    for (int currentLed = end - m_stripeStart; currentLed > start; currentLed -= space) {
                        patchStart = currentLed;
                        for (int i = m_ledStripe.length; (m_ledStripe.length != 0) && (currentLed > patchStart + m_ledStripe.length) && (currentLed >= start); currentLed--) {
                            setIndividualColor(currentLed, m_ledStripe[i]);
                            i--;
                        }
                    }
                }
            }
            if (m_stripeStart >= space + m_ledStripe.length) m_stripeStart = 0;
            else m_stripeStart++;
            m_currentStripeTime = 0;
        }
        else m_currentStripeTime++;
    }
    public void off() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }
    public void on() {
        m_isOff = false;
    }
    private int timer = 0;
    private int timeToHit = 1;
    @Override
    public void periodic() {
        if (m_isOff) return;
        if (timer >= timeToHit){ 
            timer = 0;
        }
        else {timer++; return;}
        // Updates the rainbow effect, remove to customize the leds
        rainbow();

        // Should stay here unless periodic is completly unused. 
        updateColors();
    } 
}
