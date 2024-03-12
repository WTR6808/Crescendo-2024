package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

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

    // stripeStart, stripeTimings
    private int[][] m_ledSectionTimings = {
        {3, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
    };

    private Color m_ledStripe = new Color(100,100,100);
    private int start, end, stripeStart;
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
        m_ledSecondBuffer = new AddressableLEDBuffer(Constants.LED_Constants.LED_LENGTH);

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            setIndividualColor(i, new Color(0,0,0), false);
        }

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

    public void setSectionColor(int pos1, int pos2, int red, int green, int blue, boolean writeToMainBufferBool) {
        Color m_color = new Color(red, green, blue);
        for (int i = pos1; i < pos2; i++) {
            if (writeToMainBufferBool) m_ledBuffer.setLED(i, m_color);
            else m_ledSecondBuffer.setLED(i, m_color);
        }
    }
    
    public void setSectionColor(int pos1, int pos2, Color m_color, boolean writeToMainBufferBool) {
        for (int i = pos1; i < pos2; i++) {
            if (writeToMainBufferBool) m_ledBuffer.setLED(i, m_color);
            else m_ledSecondBuffer.setLED(i, m_color);
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

    public void stripe(int stripe, int stripeLen, int space, boolean isReversed, int updateAtThisManyCycles, boolean ledSpaceOff, boolean writeToMainBufferBool) {
        if (m_ledSectionTimings[stripe][1] >= updateAtThisManyCycles) m_ledSectionTimings[stripe][1] = 0;
        else { m_ledSectionTimings[stripe][1]++; return; }
       
        start = m_ledSections[stripe][0];
        end = m_ledSections[stripe][1];
        stripeStart = m_ledSectionTimings[stripe][0];

        // led off color
        Color offColor = new Color(0, 0, 0);

        // create array for copying from
        List<Color> buf = new ArrayList<Color>();


        // Make a single stripe in the array with the stripe being in the begining of the array. 
        // example [X, X, X, Y, Y, Y, Y, Y], x = color, y = space 
        for (int i = 0; i < stripeLen; i++) buf.add(m_ledStripe);
        for (int i = stripeLen; i < stripeLen+space; i++) buf.add(offColor);


        // just zeroing the section to get rid of previous result
        setSectionColor(start, end, offColor, false);

        // Makes first part of stripe section
        // example where ledStart = 2
        // X, X, X, Y [Y, Y, X, X, X, Y, Y, Y, Y, X...]
        // everything outside the array does not get writen. 
        boolean didStart = false;

        if (!isReversed)
            for (int i = start; i < end; i += buf.size()) {
                if (stripeStart != 0 && !didStart) {
                    for (int j = space+stripeLen - stripeStart; j < buf.size(); j++) {
                        m_ledSecondBuffer.setLED(i++, buf.get(j));
                    }
                    didStart = true;
                }

                for (int j = 0; j < buf.size() && j+i <= end; j++) {
                    m_ledSecondBuffer.setLED(i+j, buf.get(j));
                }
            }

        else {
            for (int i = end; i > start; i -= buf.size()) {
                if (stripeStart != 0 && !didStart) {
                    for (int j = space+stripeLen - stripeStart; j < buf.size(); j++) {
                        m_ledSecondBuffer.setLED(i--, buf.get(j));
                    }
                    didStart = true;
                }

                for (int j = 0; j < buf.size() && i-j >= start; j++) {
                    m_ledSecondBuffer.setLED(i-j, buf.get(j));
                }
            }
        }
        m_ledSectionTimings[stripe][0]++;
        if (m_ledSectionTimings[stripe][0] >= stripeLen + space) m_ledSectionTimings[stripe][0] = 0;
    }


    // // Makes stripes apear on select led stripe
    // public void stripe(int stripe, int stripeLen, int space, boolean isReversed, int updateAtThisManyCycles, boolean ledSpaceOff, boolean writeToMainBufferBool) {
    //     start = end = buf1 = buf2 = stripeStart = currentLed = 0;

    //     if (m_ledSectionTimings[stripe][1] >= updateAtThisManyCycles) m_ledSectionTimings[stripe][1] = 0;
    //     else { m_ledSectionTimings[stripe][1]++; return; }

    //     Color setLedToOffColor = new Color(0,0,0);
        
    //     // clear secondary buffer
    //     setSectionColor(0, m_ledSecondBuffer.getLength(), setLedToOffColor, false);
    //     start = m_ledSections[stripe][0];
    //     end = m_ledSections[stripe][1];
    //     stripeStart = m_ledSectionTimings[stripe][0];

    //     // stripe setting 
    //     for (currentLed = start+stripeStart; currentLed < end; currentLed += space + stripeLen) 
    //         if (currentLed+stripeLen > end) {
    //             buf1 = currentLed+stripeLen + (currentLed+stripeLen-end);
    //             setSectionColor(currentLed, buf1, m_ledStripe, writeToMainBufferBool); 
    //         }
    //         else setSectionColor(currentLed, currentLed+stripeLen, m_ledStripe, writeToMainBufferBool);

    //     // sets spaces between stripes
    //     if (ledSpaceOff)
    //     for (currentLed = start+stripeStart; currentLed <= end; currentLed += space + stripeLen) 
    //         if (currentLed+stripeLen+space > end) {
    //             buf1 = currentLed+space -(currentLed+stripeLen+space-end);
    //             setSectionColor(currentLed+stripeLen, buf1, setLedToOffColor, writeToMainBufferBool);
    //         }
    //         else setSectionColor(currentLed+stripeLen, currentLed+stripeLen+space, setLedToOffColor, writeToMainBufferBool);
        
    //     if (stripeStart != 0)
    //         setSectionColor(start, start+stripeStart, new Color(10, 50, 10), writeToMainBufferBool);



    //     // for (int currentLed = start+stripeStart; currentLed >= start; currentLed -= space + stripeLen) 
    //     //     if (currentLed - stripeLen < start) currentLed = currentLed-stripeLen + start;
    //     //     setSectionColor(currentLed-stripeLen, currentLed, m_ledStripe, writeToMainBufferBool);


    //     if (m_ledSectionTimings[stripe][0] < space+stripeLen+1) m_ledSectionTimings[stripe][0]++;
    //     else m_ledSectionTimings[stripe][0] = 0;
    //     // m_stripeTime++;
    //     // if (m_stripeTime > space + stripeLen) m_stripeTime = 0;
    // }

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
            if (secondBufLED.red != 0 && secondBufLED.green != 0 && secondBufLED.blue != 0)
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
        // rainbow();
        setSectionColor(0, m_ledBuffer.getLength(), new Color(0,0,0), true);

        stripe(0, 3, 5, false, 10, false, false);
        // stripe(1, 3, 2, false, 10, false, false);

        // Does what it says :D
        copySecondaryBufferToMain();

        // Should stay here unless periodic is completly unused. 
        updateColors();
    } 
}
