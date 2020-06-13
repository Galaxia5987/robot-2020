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
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.LinkedHashMap;

import static frc.robot.Constants.LED.DEFAULT_COLOR;
import static frc.robot.Constants.LED.MINIMAL_DIMNESS;
import static frc.robot.Ports.LED.STRIP;
import static frc.robot.Ports.LED.STRIP_LENGTH;

/**
 * Subsystem controlling the LEDs on the robot.
 */
public class LED extends SubsystemBase {

    private final AddressableLED strip;
    private int length;
    /**
     * Creates a new LED subsystem.
     */
    public LED(int strip_length) {
        length = strip_length;
        strip = new AddressableLED(STRIP);
        strip.setLength(strip_length); //Expensive call, don't call more than once.
        start();
    }

    public void set(AddressableLEDBuffer colorsBuffer){
        strip.setData(colorsBuffer);
    }

    @Deprecated
    public void setAnimation(int frameSpeed, boolean loopAtEnd, AddressableLEDBuffer... frames){
    }

    public void clear() {

    }

    public void start(){
        strip.start();
    }

    public void stop(){
        strip.stop();
    }

    public int getLength(){
        return length;
    }
}
