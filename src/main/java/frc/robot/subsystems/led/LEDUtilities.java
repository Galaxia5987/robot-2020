package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpiutil.math.MathUtil;
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
    public static AddressableLEDBuffer colorLengths(int strip_length, ImmutablePair<Integer, Color>... colors) {
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
    public static AddressableLEDBuffer singleColor(int strip_length, Color color) {
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        for(int i = 0; i < strip_length; i++)
            colorsBuffer.setLED(i,color);
        return colorsBuffer;
    }

    public static AddressableLEDBuffer blendColors(int strip_length, boolean loop_hue, ImmutablePair<Integer, Color>... colors){
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        int b = 0;
        for(int i = 0; i < strip_length; i++){
            if(b >= colors.length) b = colors.length-1;
            if(i >= colors[b].left)
                b+=1;
            if(loop_hue){
                Color lastColor = colors[Math.floorMod(b - 1, colors.length)].right;
                Color nextColor = colors[Math.floorMod(b, colors.length)].right;
                int dist = Math.floorMod(colors[Math.floorMod(b, colors.length)].left - colors[Math.floorMod(b - 1, colors.length)].left, colors.length);
                colorsBuffer.setLED(i, blend(lastColor, nextColor, dist, Math.floorMod(i + 1 - colors[Math.floorMod(b - 1, colors.length)].left, colors.length)));
            }
            else {
                Color lastColor = colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].right;
                Color nextColor = colors[MathUtil.clamp(b, 0, colors.length - 1)].right;
                int dist = colors[MathUtil.clamp(b, 0, colors.length - 1)].left - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left;
                colorsBuffer.setLED(i, blend(lastColor, nextColor, dist, i + 1 - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left));
            }
        }
        return colorsBuffer;
    }

    private static Color blend(Color colorA, Color colorB, int dist, int current){
        double p = current / ((double)dist - 1);
        double r = colorA.red * (1 - p) + colorB.red * p;
        double g = colorA.green * (1 - p) + colorB.green * p;
        double b = colorA.blue * (1 - p) + colorB.blue * p;
        return new Color(r,g,b);

    }


    public static String colorToString(Color color) {
        return String.format("(%s, %s, %s),", color.red, color.green, color.blue);
    }

    public static void printBuffer(AddressableLEDBuffer buffer){
        System.out.print("[[");
        for(int i = 0; i < buffer.getLength(); i++)
            System.out.println(colorToString(buffer.getLED(i)));
        System.out.print("]]");
    }

}