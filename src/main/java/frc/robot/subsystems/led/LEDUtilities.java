package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.Arrays;


public class LEDUtilities {

    /**
     * Shifts a buffer by a certain offset
     * @param buffer buffer pointer
     * @param offset pixels to offset right: 101100 >2> 001011
     */
    public static void shift(AddressableLEDBuffer buffer, int offset){
        AddressableLEDBuffer temp = new AddressableLEDBuffer(buffer.getLength());
        for(int i = 0; i < buffer.getLength(); i ++) {
            temp.setLED((i + offset) % buffer.getLength(), buffer.getLED((i + offset) % buffer.getLength()));
            buffer.setLED((i + offset) % buffer.getLength(), i>=offset ? temp.getLED(i) : buffer.getLED(i));
        }
    }

    /**
     * Returns a copy of the original buffer, shifted right by an offset.
     * @param buffer original buffer
     * @param offset pixels to offset right: 101100 >2> 001011
     * @return
     */
    public static AddressableLEDBuffer getShifted(AddressableLEDBuffer buffer, int offset){
        AddressableLEDBuffer temp = new AddressableLEDBuffer(buffer.getLength());
        for(int i = 0; i < buffer.getLength(); i ++) {
            temp.setLED((i + offset) % buffer.getLength(), buffer.getLED(i));
        }
        return temp;
    }

    /**
     * Returns a mirror of a buffer along the center of it.
     * @param firstHalf true if the first half should copy over, false if the second half is copied.
     * @return
     */
    public static AddressableLEDBuffer getSymmetric(AddressableLEDBuffer buffer, boolean firstHalf){
        AddressableLEDBuffer newBuffer = new AddressableLEDBuffer(buffer.getLength());
        int m = firstHalf ? -1 : 1;

        for (int i = 0; i < newBuffer.getLength(); i ++){
            newBuffer.setLED(i, buffer.getLED( (int)Math.round((buffer.getLength()-1 + m * Math.abs(2 * i - buffer.getLength()-1))/2.) ));
        }
        return newBuffer;
    }
    /**
     * Sets the colors of the strip with mapping between lengths to each color.
     *
     * For example, the map [1: blue, 4: red, 2: green] will set the first cell to blue, the subsequent 4 cells to red
     * and the subsequent 2 to green.
     * Missing cells will copy over the last color, and excessive cells will be ignored.
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
                colorsBuffer.setLED(i,
                        blend(
                                lastColor,
                                nextColor,
                                Math.floorMod(colors[Math.floorMod(b, colors.length)].left - colors[Math.floorMod(b - 1, colors.length)].left + 1, strip_length),
                                Math.floorMod(i - colors[Math.floorMod(b - 1, colors.length)].left , strip_length))
                );
            }
            else {
                Color lastColor = colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].right;
                Color nextColor = colors[MathUtil.clamp(b, 0, colors.length - 1)].right;
                int dist = colors[MathUtil.clamp(b, 0, colors.length - 1)].left - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left + 1;
                colorsBuffer.setLED(i, blend(lastColor, nextColor, dist, i - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left));
            }
        }
        return colorsBuffer;
    }

    public static AddressableLEDBuffer hsvBlendColors(int strip_length, boolean loop_hue, ImmutablePair<Integer, Color>... colors){
        AddressableLEDBuffer colorsBuffer = new AddressableLEDBuffer(strip_length);
        int b = 0;
        for(int i = 0; i < strip_length; i++){
            if(b >= colors.length) b = colors.length-1;
            if(i >= colors[b].left)
                b+=1;
            if(loop_hue){
                Color lastColor = colors[Math.floorMod(b - 1, colors.length)].right;
                Color nextColor = colors[Math.floorMod(b, colors.length)].right;
                int current = Math.floorMod(i + 1 - colors[Math.floorMod(b - 1, colors.length)].left , strip_length);
                colorsBuffer.setLED(i,
                        HSVblend(
                                lastColor,
                                nextColor,
                                Math.floorMod(colors[Math.floorMod(b, colors.length)].left - colors[Math.floorMod(b - 1, colors.length)].left + 1, strip_length),
                                Math.floorMod(i - colors[Math.floorMod(b - 1, colors.length)].left , strip_length))
                );
            }
            else {
                Color lastColor = colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].right;
                Color nextColor = colors[MathUtil.clamp(b, 0, colors.length - 1)].right;
                int dist = colors[MathUtil.clamp(b, 0, colors.length - 1)].left - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left + 1;
                colorsBuffer.setLED(i, HSVblend(
                        lastColor,
                        nextColor,
                        dist,
                        i - colors[MathUtil.clamp(b - 1, 0, colors.length - 1)].left)
                );
            }
        }
        return colorsBuffer;
    }

    public static AddressableLEDBuffer dimStrip(AddressableLEDBuffer strip, double dimPercent){
        AddressableLEDBuffer newStrip = new AddressableLEDBuffer(strip.getLength());
        for(int i = 0; i < strip.getLength(); i++){
            newStrip.setLED(i, new Color(
                    strip.getLED(i).red * dimPercent,
                    strip.getLED(i).green * dimPercent,
                    strip.getLED(i).blue * dimPercent)
            );
        }
        return newStrip;
    }

    public static AddressableLEDBuffer rotateStrip(AddressableLEDBuffer strip, int pixels_right){
        AddressableLEDBuffer newStrip = new AddressableLEDBuffer(strip.getLength());
        for(int i = 0; i < strip.getLength(); i++){
            newStrip.setLED(i, strip.getLED(Math.floorMod(i - pixels_right, strip.getLength())));
        }
        return newStrip;
    }

    private static Color blend(Color colorA, Color colorB, int dist, int current){
        double p = current / ((double)dist - 1);
        double r = colorA.red * (1 - p) + colorB.red * p;
        double g = colorA.green * (1 - p) + colorB.green * p;
        double b = colorA.blue * (1 - p) + colorB.blue * p;
        return new Color(r,g,b);

    }

    private static Color HSVblend(Color colorA, Color colorB, int dist, int current){
        double[] colorAHSV = HSV.rgb2hsv(colorA.red, colorA.green,colorA.blue);
        double[] colorBHSV = HSV.rgb2hsv(colorB.red, colorB.green,colorB.blue);

        System.out.println(current + "/" + dist);
        double p;
        if(dist != 1)
            p = current / ((double)dist - 1);
        else
            p = 1;
        int h;
        if(Math.abs(colorAHSV[0] - colorBHSV[0]) <= 90 )
            h = (int)((colorAHSV[0] * (1 - p) + colorBHSV[0] * p));
        else {
            if(colorAHSV[0] < colorBHSV[0])
                h = Math.floorMod((int)((colorAHSV[0] * (1 - p) + (colorBHSV[0]-180) * p)), 180);
            else
                h = Math.floorMod((int)(((colorAHSV[0]-180) * (1 - p) + colorBHSV[0] * p)), 180);
        }
        double s = colorAHSV[1] * (1 - p) + colorBHSV[1] * p;
        double v = colorAHSV[2] * (1 - p) + colorBHSV[2] * p;
        double[] rgb = HSV.hsv2rgb(h, s, v);
        return new Color(rgb[0], rgb[1], rgb[2]);
    }

    public static String colorToString(Color color) {
        return String.format("%s, %s, %s", color.red, color.green, color.blue);
    }

    public static void printBuffer(AddressableLEDBuffer buffer){
        for(int i = 0; i < buffer.getLength(); i++)
            System.out.println(colorToString(buffer.getLED(i)));
    }

    public static void printHSV(AddressableLEDBuffer buffer){
        for(int i = 0; i < buffer.getLength(); i++){
            System.out.println(Arrays.toString(HSV.rgb2hsv(buffer.getLED(i).red, buffer.getLED(i).green, buffer.getLED(i).blue)));
        }
    }

}
