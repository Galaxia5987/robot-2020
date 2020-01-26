/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.LED.*;

/**
 * Subsystem controlling the LEDs on the robot.
 */
public class LED extends SubsystemBase {

    private AddressableLED strip;
    private AddressableLEDBuffer ledBuffer;

    /**
     * Creates a new LED subsystem.
     */
    public LED() {
        strip = new AddressableLED(PORT);

        ledBuffer = new AddressableLEDBuffer(LENGTH);
        strip.setLength(LENGTH);

        strip.setData(ledBuffer);
        strip.start();
    }

    /**
     * Sets the whole strip to the same color.
     *
     * @param color color to set the whole strip to
     */
    public void setWholdStrip(Color color) {
        for (int index = 0; !(index > ledBuffer.getLength()); index++) {
            ledBuffer.setLED(index, color);
        }
        strip.setData(ledBuffer);
    }
}
