package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import org.apache.commons.lang3.tuple.ImmutablePair;

public class LEDUtilities {

    /**
     * Sets the colors of the strip with mapping between lengths to each color.
     *
     * For example, the map [1: blue, 4: red, 2: green] will set the first cell to blue, the subsequent 4 cells to red
     * and the subsequent 2 to green.
     * Missing cells will stay in the color they were before, and excessive cells will be ignored.
     *
     * @param colors list of pairs that maps between length to each color
     */
    public static AddressableLEDBuffer setColorLengths(int strip_length, ImmutablePair<Integer, Color>... colors) {
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        int colorIndex = 0;
        int colorSum = 0;
        for(int i = 0; i < strip_length; i++){
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
        return colorsBuffer;
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
    public static AddressableLEDBuffer colorRatios(int strip_length, ImmutablePair<Double, Color>... colors){
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        double total = 0;
        for(ImmutablePair<Double, Color> color : colors){
            total += color.left;
        }
        int weightIndex = 0;
        double prevWeights = 0;
        for(int i = 0; i < strip_length; i++){
            if(i / (double)strip_length >= (colors[weightIndex].left + prevWeights) / total) {
                prevWeights += colors[weightIndex].left;
                weightIndex += 1;
            }
            colorsBuffer.setLED(i, colors[weightIndex].right);
        }
        return colorsBuffer;
    }
    /**
     * Sets the whole strip to a given color.
     *
     * @param color color to set the whole strip to
     */
    public AddressableLEDBuffer singleColor(int strip_length, Color color) {
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        for(int i = 0; i < strip_length; i++)
            colorsBuffer.setLED(i,color);
        return colorsBuffer;
    }
}
