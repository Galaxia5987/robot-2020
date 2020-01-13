package frc.robot.subsystems.ColorWheel;

import org.junit.Test;

import java.util.Scanner;

public class ColorWheelAlgorithmTest {
    private static Scanner kb = new Scanner(System.in);
    private String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String[] input = {
            "Yellow", "Red", "Green", "Blue",
            "Yellow", "Red", "Green", "Blue",
            "Yellow", "Red", "Green", "Blue",
            "Yellow", "Red", "Green", "Blue",
            "Yellow", "Red", "Green", "Blue",
            "Green", "Red", "Green", "Blue"};

    public static int indexOf(String color) {
        switch (color) {
            case ("Yellow"):
                return 0;
            case ("Red"):
                return 1;
            case ("Green"):
                return 2;
            case ("Blue"):
                return 3;
            default:
                return 500;
        }
    }

    @Test
    public void calculateSpins() throws Exception {
        String clockWisecurrentColor = "Yellow";
        String counterClockWiseCurrentColor = "Yellow";
        double clockWiseSpins = 0;
        double counterClockWiseSpins = 0;
        String matchedColor = "";
        int index = 0;
        while (clockWiseSpins < 3 && index < input.length) {
            matchedColor = input[index];
            if (indexOf(matchedColor) == indexOf(clockWisecurrentColor) + 1 || indexOf(matchedColor) == indexOf(clockWisecurrentColor) % 3) {
                clockWiseSpins += 0.125;
                clockWisecurrentColor = matchedColor;
            }
            else if (indexOf(matchedColor) == indexOf(counterClockWiseCurrentColor) - 1 || indexOf(matchedColor) % 3 == indexOf(counterClockWiseCurrentColor)) {
                counterClockWiseSpins += 0.125;
                clockWisecurrentColor = matchedColor;
            }
            index++;

        }
        System.out.println(clockWiseSpins);
        System.out.println(counterClockWiseSpins);


    }
}
