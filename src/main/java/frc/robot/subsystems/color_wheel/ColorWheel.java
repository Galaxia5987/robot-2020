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
import frc.robot.RobotContainer;
import frc.robot.subsystems.color_wheel.commands.ManualControl;

import static frc.robot.Ports.ColorWheel.IS_MOTOR_REVERSED;

public class ColorWheel extends SubsystemBase {

    //Color sensor definitions
    private final VictorSPX motor = new VictorSPX(Ports.ColorWheel.MOTOR);
    public final I2C.Port i2cPort = I2C.Port.kOnboard;
    public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private String colorString = "";
    private final ColorMatch colorMatcher = new ColorMatch();
    private final Color BlueTarget = ColorMatch.makeColor(Constants.ColorWheel.BLUE_RGB[0], Constants.ColorWheel.BLUE_RGB[1], Constants.ColorWheel.BLUE_RGB[2]);
    private final Color GreenTarget = ColorMatch.makeColor(Constants.ColorWheel.GREEN_RGB[0], Constants.ColorWheel.GREEN_RGB[1], Constants.ColorWheel.GREEN_RGB[2]);
    private final Color RedTarget = ColorMatch.makeColor(Constants.ColorWheel.RED_RGB[0], Constants.ColorWheel.RED_RGB[1], Constants.ColorWheel.RED_RGB[2]);
    private final Color YellowTarget = ColorMatch.makeColor(Constants.ColorWheel.YELLOW_RGB[0], Constants.ColorWheel.YELLOW_RGB[1], Constants.ColorWheel.YELLOW_RGB[2]);

    private ColorMatchResult match;

    private boolean manual = false;

    public ColorWheel() {
        motor.setInverted(IS_MOTOR_REVERSED);
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public String getColorString() {
        return colorString;
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public boolean isManual(){
        return manual;
    }

    public int indexOfColor(String color) {

        switch (color.charAt(0)) {
            case ('Y'):
                return 0;
            case ('R'):
                return 1;
            case ('G'):
                return 2;
            case ('B'):
                return 3;
            default:
                throw new IllegalArgumentException(String.format("Color %s does not have an index", color));

        }
    }

    /*
    Returns the currently detected color as a string
     */
    private String colorToString() {
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


    public void setPower(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void periodic() {
        match = colorMatcher.matchClosestColor(colorSensor.getColor());
        Color detectedColor = colorSensor.getColor();
        colorString = colorToString();


        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
    }
}
