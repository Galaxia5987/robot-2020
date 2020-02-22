package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SmartDash {

    public static void putNumber(String key, double number) {
        if(Robot.debug)
            SmartDashboard.putNumber(key, number);
    }

}
