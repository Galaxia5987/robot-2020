package frc.robot.subsystems.led.present_leds;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LEDUtilities;
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.awt.*;
import java.io.*;
import java.util.Arrays;

class Save
{
    public static void main(String args[])
    {
        try{
            // Create file
            FileWriter fstream = new FileWriter("src\\main\\java\\frc\\robot\\subsystems\\led\\present_leds\\" + "strip.txt");
            BufferedWriter out = new BufferedWriter(fstream);
            AddressableLEDBuffer buffer = LEDUtilities.blendColors(22, true,
                    new ImmutablePair<>(0, Color.kRed),
                    new ImmutablePair<>(5, HSV.hsv2Color( 360 * 5 / 22., 1, 1)),
                    new ImmutablePair<>(11, Color.kCyan),
                    new ImmutablePair<>(17, HSV.hsv2Color(360*17/22., 1, 1))
            );
            LEDUtilities.printBuffer(LEDUtilities.getShifted(LEDUtilities.colorLengths(22,
                    new ImmutablePair<>(1, Color.kRed),
                    new ImmutablePair<>(21, Color.kBlack)), 2));
            for (int i = 0; i < buffer.getLength(); i++){
                System.out.println(i + ", " + HSV.rgb2hsv(buffer.getLED(i).red, buffer.getLED(i).green, buffer.getLED(i).blue)[0]);
                out.write(LEDUtilities.colorToString(buffer.getLED(i)));
                out.write("\n");
            }

            //Close the output stream
            out.close();
        }catch (Exception e){//Catch exception if any
            System.err.println("Error: " + e.getMessage());
        }
    }
}

class SaveGif
{
    public static void main(String args[])
    {
        try{
            // Create file
            FileWriter fstream = new FileWriter("src\\main\\java\\frc\\robot\\subsystems\\led\\present_leds\\" + "animation.txt");
            BufferedWriter out = new BufferedWriter(fstream);
            AddressableLEDBuffer[] buffer = new AddressableLEDBuffer[]{LEDUtilities.colorLengths(22,
                    new ImmutablePair<>(5, edu.wpi.first.wpilibj.util.Color.kRed),
                    new ImmutablePair<>(6, Color.kLime),
                    new ImmutablePair<>(5, Color.kCyan),
                    new ImmutablePair<>(6, Color.kPurple))};

            for(int a = 0; a < buffer.length; a++) {
                for (int i = 0; i < buffer[a].getLength(); i++) {
                    out.write(LEDUtilities.colorToString(buffer[a].getLED(i)));
                    out.write("\n");
                }
                if(a+1 < buffer.length) out.write("---\n");
            }
            //Close the output stream
            out.close();
        }catch (Exception e){//Catch exception if any
            System.err.println("Error: " + e.getMessage());
        }
    }
}