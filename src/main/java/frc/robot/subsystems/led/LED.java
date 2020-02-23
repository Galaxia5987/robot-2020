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
    private AddressableLEDBuffer colorsBuffer;

    /**
     * Creates a new LED subsystem.
     */
    public LED() {
        strip = new AddressableLED(STRIP);
        colorsBuffer = new AddressableLEDBuffer(STRIP_LENGTH);
        // Set the color of the LEDs to Galaxia blue at startup.
        setWholeStrip(DEFAULT_COLOR);
        strip.setLength(STRIP_LENGTH); //Expensive call, don't call more than once.
        start();
    }

    /**
     * Sets the colors of the strip with mapping between lengths to each color.
     *
     * For example, the map [1: blue, 4: red, 2: green] will set the first cell to blue, the subsequent 4 cells to red
     * and the subsequent 2 to green.
     * Missing cells will stay in the color they were before, and excessive cells will be ignored.
     *
     * @param colors list of pairs that maps between length to each color
     */
    public final void setColorLengths(ImmutablePair<Integer, Color>... colors) {
        int colorIndex = 0;
        int colorSum = 0;
        for(int i = 0; i < colorsBuffer.getLength(); i++){
            Color color;
            if(colors[colorIndex].left + colorSum < i + 1){
                if (colorIndex + 1 >= colors.length)
                    color = Color.kBlack;
                else{
                    colorSum += colors[colorIndex].left;
                    colorIndex ++;
                    color = colors[colorIndex].right;
                }
            }
            else
                color = colors[colorIndex].right;
            colorsBuffer.setLED(i, color);
        }

        strip.setData(colorsBuffer);
    }

    /**
     * Sets the colors of the strip with mapping between length ratios to each color.
     *
     * The map maps between parts of the strip to colors. For example, if the strip's length is 20, the map
     * [0.3: blue, 0.5: red, 0.2: green] will set the first 6 (20 * 0.3) cell to blue, the subsequent 10 (20 * 0.5)
     * cells to red, and the subsequent 4 (20 * 0.2) cells to green.
     * Missing cells will stay in the color they were before, and excessive cells will be ignored.
     *
     * @param colors map that maps between ratio of the strip length to each color
     */
    @SafeVarargs
    public final void setColorRatios(ImmutablePair<Double, Color>... colors){
        double total = 0;
        for(ImmutablePair<Double, Color> color : colors){
            total += color.left;
        }
        int weightIndex = 0;
        double prevWeights = 0;
        for(int i = 0; i < colorsBuffer.getLength(); i++){
            if(i / (double)colorsBuffer.getLength() >= (colors[weightIndex].left + prevWeights) / total) {
                prevWeights += colors[weightIndex].left;
                weightIndex += 1;
            }
            colorsBuffer.setLED(i, colors[weightIndex].right);
        }
        strip.setData(colorsBuffer);
    }
    /**
     * Sets the whole strip to a given color.
     *
     * @param color color to set the whole strip to
     */
    public void setWholeStrip(Color color) {
        //setColorLengths(new ImmutablePair<>(colorsBuffer.getLength(), color));
        for(int i = 0; i < colorsBuffer.getLength(); i++)
            colorsBuffer.setLED(i,color);
    }

    public void setAnimation(int frameSpeed, boolean loopAtEnd, AddressableLEDBuffer... frames){
    }

    public void clear(){

    }
    public AddressableLEDBuffer getColorsBuffer(){
        return colorsBuffer;
    }

    public void start(){
        strip.start();
    }

    public void stop(){
        strip.stop();
    }
}
