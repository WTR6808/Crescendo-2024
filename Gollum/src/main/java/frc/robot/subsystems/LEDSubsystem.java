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
    private static AddressableLEDBuffer m_ledSecondBuffer = null;

    // Main light interval update timer! dont touch *timer* just *timeToHit*
    private int timer = 0;
    private int timeToHit = 1;

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
        m_ledSecondBuffer = m_ledBuffer; 

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

    public void setIndividualColor(int pos, int red, int green, int blue, boolean useMainBuffer) {
        Color m_color = new Color(red, green, blue);
        if (useMainBuffer) m_ledBuffer.setLED(pos, m_color);
        else m_ledSecondBuffer.setLED(pos, m_color);
    }
  
    public void setIndividualColor(int pos, Color m_color, boolean useMainBuffer) {
        if (useMainBuffer) m_ledBuffer.setLED(pos, m_color);
        else m_ledSecondBuffer.setLED(pos, m_color);    
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

    // Makes stripes apear on select led stripes
    public void stripe(int[] stripes, int space, boolean[] reverse, int updateAtThisManyCycles, boolean writeToMainBufferBool) {
        if (updateAtThisManyCycles != m_stripeTime) m_stripeTime = updateAtThisManyCycles;
        if (m_currentStripeTime == m_stripeTime) {
            int stripe, start, end, patchStart;
            int stripeLen = m_ledStripe.length;
            Color setLedToOffColor = new Color(0, 0, 0);
            for (int n = 0; n < stripes.length; n++) {
                stripe = stripes[n];
                start = m_ledSections[stripe][0];
                end = m_ledSections[stripe][1];
                // not reversed
                if (!reverse[stripe]) 
                    for (int currentLed = start; currentLed >= start && currentLed <= end; currentLed++) {
                        for (int i = 0; i < stripeLen && currentLed+i-1 <= end; i++) {
                            setIndividualColor(currentLed, m_ledStripe[i], writeToMainBufferBool);
                            currentLed++;
                        }
                        for (int i = 0; i < space; i++) 
                            setIndividualColor(currentLed+i, setLedToOffColor, writeToMainBufferBool);
                        
                        currentLed += space - 1;
                    }
                else {
                    for (int currentLed = end; currentLed <= end && currentLed >= start; currentLed--) {
                        for (int i = 0; i < stripeLen && currentLed-i-1 >= start; i++) {
                            setIndividualColor(i, m_ledStripe[i], false);
                            currentLed--;
                        }
                        for (int i = 0; i < space; i++) 
                            setIndividualColor(currentLed-i, setLedToOffColor, writeToMainBufferBool);

                        currentLed -= space + 1;
                    }
                }


                // if (!reverse[n]) {
                //     for (int i = start; i >= start+m_stripeStart-space; i++) 
                //         setIndividualColor(start+i, m_ledStripe[m_ledStripe.length-1 - i]);
                //     for (int currentLed = start + m_stripeStart; currentLed < end; currentLed += space) {
                //         patchStart = currentLed;
                //         for (int i = 0; (m_ledStripe.length != 0) && (currentLed < patchStart + m_ledStripe.length) && (currentLed <= end); currentLed++) {
                //             setIndividualColor(currentLed, m_ledStripe[i]);
                //             i++;
                //         }
                //     }
                // }
                // else {
                //     for (int currentLed = end - m_stripeStart; currentLed > start; currentLed -= space) {
                //         for (int i = start; i >= start+m_stripeStart-space; i++) 
                //             setIndividualColor(start+i, m_ledStripe[m_ledStripe.length-1 - i]);

                //         patchStart = currentLed;
                //         for (int i = m_ledStripe.length; (m_ledStripe.length != 0) && (currentLed > patchStart + m_ledStripe.length) && (currentLed >= start); currentLed--) {
                //             setIndividualColor(currentLed, m_ledStripe[i]);
                //             i--;
                //         }
                //     }
                // }
            }
            if (m_stripeStart >= space + m_ledStripe.length - 1) m_stripeStart = 0;
            else m_stripeStart++;
            m_currentStripeTime = 0;
        }
        else m_currentStripeTime++;
    }

    // Turns off leds and clears buffers
    public void off() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_ledSecondBuffer.setRGB(i, 0, 0, 0);
        }
        m_isOff = true;
    }
    // Turn on leds!
    public void on() {
        m_isOff = false;
        setAllColors(100, 100, 100);
        updateColors();
    }
    
    // Does what it says :D
    public void copySecondaryBufferToMain() {
        Color secondBufLED;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            secondBufLED = m_ledSecondBuffer.getLED(i);
            if (secondBufLED.red == 0 && secondBufLED.green == 0 && secondBufLED.blue == 0)
                m_ledBuffer.setLED(i, m_ledSecondBuffer.getLED(i));
        }
    }
    
    @Override
    public void periodic() {
        if (m_isOff) return;
        if (timer >= timeToHit){ 
            timer = 0;
        }
        else {timer++; return;}
        // Updates the rainbow effect, remove to customize the leds
        rainbow();

        int[] stripes = {0, 2};
        boolean[] reverse = {false, false}; 
        stripe(stripes, 5, reverse, 5, false);
        // Does what it says :D
        copySecondaryBufferToMain();

        // Should stay here unless periodic is completly unused. 
        updateColors();
    } 
}
