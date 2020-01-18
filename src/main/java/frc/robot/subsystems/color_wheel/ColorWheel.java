/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.color_wheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class ColorWheel extends SubsystemBase {

    //Color sensor definitions
    public final I2C.Port i2cPort = I2C.Port.kOnboard;
    public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private String colorString = "";
    private final ColorMatch colorMatcher = new ColorMatch();
    private final Color BlueTarget = ColorMatch.makeColor(Constants.ColorWheel.BLUE_RGB[0], Constants.ColorWheel.BLUE_RGB[1], Constants.ColorWheel.BLUE_RGB[2]);
    private final Color GreenTarget = ColorMatch.makeColor(Constants.ColorWheel.GREEN_RGB[0], Constants.ColorWheel.GREEN_RGB[1], Constants.ColorWheel.GREEN_RGB[2]);
    private final Color RedTarget = ColorMatch.makeColor(Constants.ColorWheel.RED_RGB[0], Constants.ColorWheel.RED_RGB[1], Constants.ColorWheel.RED_RGB[2]);
    private final Color YellowTarget = ColorMatch.makeColor(Constants.ColorWheel.YELLOW_RGB[0], Constants.ColorWheel.YELLOW_RGB[1], Constants.ColorWheel.YELLOW_RGB[2]);

    private final VictorSPX spinMotor = new VictorSPX(Ports.ColorWheel.MOTOR);

    public ColorWheel() {
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);
    }

    public String getColorString() {
        return colorString;
    }

    public int indexOfColor(String color) {
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
                throw new IllegalArgumentException(String.format("Color %s does not have an index", color));

        }
    }

    private String colorToString(Color color) {
        ColorMatchResult match = colorMatcher.matchClosestColor(color);
        String colorInString;
        if (match.color == BlueTarget) {
            colorInString = "Blue";
        } else if (match.color == RedTarget) {
            colorInString = "Red";
        } else if (match.color == GreenTarget) {
            colorInString = "Green";
        } else if (match.color == YellowTarget) {
            colorInString = "Yellow";
        } else {
            colorInString = "Unknown";
        }
        return colorInString;
    }

    public void setNeutralMode(NeutralMode mode){
        spinMotor.setNeutralMode(mode);
    }

    public void setMotorSpeed(double percent) {
        spinMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();
        colorString = colorToString(detectedColor);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putString("Detected Color", colorString);
    }
}
