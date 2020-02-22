/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.Constants.LED.DIMNESS_INCREASE;

/**
 * Buffer storage for Addressable LEDs.
 */
public class ColorsBuffer extends AddressableLEDBuffer {
    byte[] m_buffer;

    /**
     * Constructs a new LED buffer with the specified length.
     *
     * @param length     The length of the buffer in pixels
     */
    public ColorsBuffer(int length) {
        super(length);
        this.m_buffer = new byte[length * 4];
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
    }
}
