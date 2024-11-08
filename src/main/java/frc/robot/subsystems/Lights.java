package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class Lights {
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private final int[][] colors = {{255, 0, 0},
                                    {255, 128, 0},
                                    {255,255,0},
                                    {0,255,0},
                                    {0,0,255},
                                    {127,0,255},
                                    {255, 0, 255}};
    private int ColorSelect = 0;
    private int m_rainbowFirstPixelHue = 42;

    public Lights(){
        m_led.setLength(m_ledBuffer.getLength());
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setColor(){
        if (ColorSelect > 6){
            ColorSelect = 0;
        }
        for (var i = 0; i < m_ledBuffer.getLength(); i ++) {
            m_ledBuffer.setRGB(i, colors[ColorSelect][0], colors[ColorSelect][1], colors[ColorSelect][2]);
        }
        m_led.setData(m_ledBuffer);
        ColorSelect += 1;
    }

    public void setBlue(int seperation){
        for (var i = 0; i < m_ledBuffer.getLength(); i += seperation) {
            m_ledBuffer.setRGB(i, 21, 244, 238);
        }
        m_led.setData(m_ledBuffer);
    }

    public void rainbow(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }
}
