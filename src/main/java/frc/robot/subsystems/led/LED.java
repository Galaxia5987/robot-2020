/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.LED.*;

/**
 * Subsystem controlling the LEDs on the robot.
 */
public class LED extends SubsystemBase {

    private AddressableLED strip;
    /**
     * Creates a new LED subsystem.
     */
    public LED() {
        strip = new AddressableLED(PORT);

        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LENGTH);
        strip.setLength(LENGTH);

        strip.setData(ledBuffer);
        strip.start();
    }
}
