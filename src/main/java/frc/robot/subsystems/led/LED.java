/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedHashMap;

import static frc.robot.Constants.LED.DEFAULT_COLOR;
import static frc.robot.Constants.LED.MINIMAL_DIMNESS;
import static frc.robot.Ports.LED.STRIP;
import static frc.robot.Ports.LED.STRIP_LENGTH;

/**
 * Subsystem controlling the LEDs on the robot.
 */
public class LED extends SubsystemBase {

    private AddressableLED strip;
    private AddressableLEDBuffer colorsBuffer;

    /**
     * Creates a new LED subsystem.
     */
    public LED() {
        strip = new AddressableLED(STRIP);

        colorsBuffer = new AddressableLEDBuffer(STRIP_LENGTH, MINIMAL_DIMNESS);
        // Set the color of the LEDs to Galaxia blue at startup.
        setWholeStrip(DEFAULT_COLOR);
        strip.setLength(STRIP_LENGTH);

        strip.setData(colorsBuffer);
        strip.start();
    }

    /**
     * Sets the colors of the strip with mapping between lengths to each color.
     *
     * For example, the map [1: blue, 4: red, 2: green] will set the first cell to blue, the subsequent 4 cells to red
     * and the subsequent 2 to green.
     * Missing cells will stay in the color they were before, and excessive cells will be ignored.
     *
     * @param colorMap map that maps between length to each color
     */
    public void setColorLengths(LinkedHashMap<Integer, Color> colorMap) {
        int runningIndex = 0;
        for (Integer key : colorMap.keySet()) {
            int innerIndex;
            for (innerIndex = runningIndex + 1; innerIndex <= runningIndex + key; innerIndex++) {
                colorsBuffer.setLED(innerIndex, colorMap.get(key));
                if (innerIndex > colorsBuffer.getLength()) { // Exit the method if the given buffer is too long.
                    return;
                }
            }
            runningIndex = innerIndex;
        }
    }

    /**
     * Sets the colors of the strip with mapping between length ratios to each color.
     *
     * The map maps between parts of the strip to colors. For example, if the strip's length is 20, the map
     * [0.3: blue, 0.5: red, 0.2: green] will set the first 6 (20 * 0.3) cell to blue, the subsequent 10 (20 * 0.5)
     * cells to red, and the subsequent 4 (20 * 0.2) cells to green.
     * Missing cells will stay in the color they were before, and excessive cells will be ignored.
     *
     * @param colorMap map that maps between ratio of the strip length to each color
     */
    public void setColorRatios(LinkedHashMap<Double, Color> colorMap) {
        int runningIndex = 0;
        for (Double key : colorMap.keySet()) {
            int innerIndex;
            for (innerIndex = runningIndex + 1; innerIndex <= runningIndex + key * colorsBuffer.getLength(); innerIndex++) {
                colorsBuffer.setLED(innerIndex, colorMap.get(key));
                if (innerIndex > colorsBuffer.getLength()) { // Exit the method if the given buffer is too long.
                    return;
                }
            }
            runningIndex = innerIndex;
        }
    }


    /**
     * Sets the whole strip to a given color.
     *
     * @param color color to set the whole strip to
     */
    public void setWholeStrip(Color color) {
        final LinkedHashMap<Integer, Color> colorMap = new LinkedHashMap<>();
        colorMap.put(colorsBuffer.getLength(), color);
        setColorLengths(colorMap);
    }

    @Override
    public void periodic() {
        strip.setData(colorsBuffer);
    }

    /**
     * @return the current colors buffer.
     */
    public AddressableLEDBuffer getCurrentBuffer() {
        return colorsBuffer;
    }

    /**
     * Sets the colors buffer.
     *
     * @param colorBuffer colors buffer to set to
     */
    public void setColorBuffer(AddressableLEDBuffer colorBuffer) {
        colorsBuffer = colorBuffer;
    }

    public void activateDimness(boolean activate) {
        colorsBuffer.activateDimness(activate);
    }
}
