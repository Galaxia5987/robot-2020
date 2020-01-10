package frc.robot.subsystems.ColorWheel;

import org.junit.Test;
import edu.wpi.first.wpilibj.I2C;

public class ColorWheelAlgorithmTest {
    private ColorWheelAlgorithm cwa = new ColorWheelAlgorithm();
    private String[] colors = {"Yellow", "Red", "Green", "Blue"};
    @Test
    public void calculateSpins() throws Exception{
        String initialColor = "Yellow";
        int halfSpinCounter = 0;
        String matchedColor = "";
        int index = 0;
        while (halfSpinCounter < 3){
            matchedColor = colors[index%4];
            index++;
            if (matchedColor.equals(initialColor))
                halfSpinCounter ++;
        }
        System.out.println(index);
        System.out.println(halfSpinCounter);



    }
}
