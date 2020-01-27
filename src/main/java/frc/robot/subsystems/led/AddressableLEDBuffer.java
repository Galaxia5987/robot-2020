/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.led;

import static frc.robot.Constants.LED.DIMNESS_INCREASE;

/**
 * Buffer storage for Addressable LEDs.
 */
public class AddressableLEDBuffer extends edu.wpi.first.wpilibj.AddressableLEDBuffer {
    byte[] m_buffer;

    private double minDimness;
    private double currentDimness;
    private boolean dimnessActivated;

    /**
     * Constructs a new LED buffer with the specified length.
     *
     * @param length     The length of the buffer in pixels
     * @param minDimness The minimal dimness of the LEDs strip
     */
    public AddressableLEDBuffer(int length, double minDimness) {
        super(length);
        this.m_buffer = new byte[length * 4];
        this.minDimness = minDimness;
        this.currentDimness = minDimness;
        dimnessActivated = true;

    }

    /**
     * Sets the minimal dimness of the strip.
     *
     * @param minDimness minimal dimness to set the strip to
     */
    public void setMinDimness(double minDimness) {
        this.minDimness = minDimness;
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
        if (dimnessActivated) {
            currentDimness += DIMNESS_INCREASE;
            if (currentDimness > 1) {
                currentDimness = minDimness;
            }
        } else {
            currentDimness = 1;
        }
        m_buffer[index * 4] = (byte) (b * currentDimness);
        m_buffer[(index * 4) + 1] = (byte) (g * currentDimness);
        m_buffer[(index * 4) + 2] = (byte) (r * currentDimness);
        m_buffer[(index * 4) + 3] = 0;
    }

    /**
     * Activates the LEDs dimness effect.
     *
     * @param activate whether to activate the dimness effect
     */
    public void activateDimness(boolean activate) {
        dimnessActivated = activate;
    }
}
