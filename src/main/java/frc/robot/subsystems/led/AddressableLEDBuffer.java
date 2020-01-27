/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Buffer storage for Addressable LEDs.
 */
public class AddressableLEDBuffer extends edu.wpi.first.wpilibj.AddressableLEDBuffer {
    byte[] m_buffer;
    private Color[] rgbBuffer;

    /**
     * Constructs a new LED buffer with the specified length.
     *
     * @param length The length of the buffer in pixels
     */
    public AddressableLEDBuffer(int length) {
        super(length);
        m_buffer = new byte[length * 4];
        rgbBuffer = new Color[length];
    }

    /**
     * Sets a specific led in the buffer.
     *
     * @param index the index to write
     * @param r     the r value [0-255]
     * @param g     the g value [0-255]
     * @param b     the b value [0-255]
     */
    @SuppressWarnings("ParameterName")
    public void setRGB(int index, int r, int g, int b) {
        m_buffer[index * 4] = (byte) b;
        m_buffer[(index * 4) + 1] = (byte) g;
        m_buffer[(index * 4) + 2] = (byte) r;
        m_buffer[(index * 4) + 3] = 0;
        rgbBuffer[index] = new Color(r, g, b);
    }

    /**
     * @return array of the current colors on the buffer in the RGB format
     */
    public Color[] getCurrentBuffer() {
        return rgbBuffer;
    }
}
