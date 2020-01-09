/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ColorWheel;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorWheel extends SubsystemBase {

  public I2C.Port i2cPort = I2C.Port.kOnboard;
  public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor;
  private int Counter = 0;
  private String lastColorString = "";
  private String colorString = "";
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color BlueTarget = ColorMatch.makeColor(Constants.ColorWheel.BLUE_RGB[0], Constants.ColorWheel.BLUE_RGB[1], Constants.ColorWheel.BLUE_RGB[2]);
  private final Color GreenTarget = ColorMatch.makeColor(Constants.ColorWheel.GREEN_RGB[0], Constants.ColorWheel.GREEN_RGB[1], Constants.ColorWheel.GREEN_RGB[2]);
  private final Color RedTarget = ColorMatch.makeColor(Constants.ColorWheel.RED_RGB[0], Constants.ColorWheel.RED_RGB[1], Constants.ColorWheel.RED_RGB[2]);
  private final Color YellowTarget = ColorMatch.makeColor(Constants.ColorWheel.YELLOW_RGB[0], Constants.ColorWheel.YELLOW_RGB[1], Constants.ColorWheel.YELLOW_RGB[2]);



  /**
   * Creates a new ExampleSubsystem.
   */
  public ColorWheel() {
    colorMatcher.addColorMatch(BlueTarget);
    colorMatcher.addColorMatch(GreenTarget);
    colorMatcher.addColorMatch(RedTarget);
    colorMatcher.addColorMatch(YellowTarget);


  }

  public void update_sensor_encoder(){
    if (!lastColorString.equals(colorString))
      Counter+=1;
    lastColorString = colorString;
  }

  @Override
  public void periodic() {
    detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if (match.color == BlueTarget) {
      colorString = "Blue";
    } else if (match.color == RedTarget) {
      colorString = "Red";
    } else if (match.color == GreenTarget) {
      colorString = "Green";
    } else if (match.color == YellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    update_sensor_encoder();
    SmartDashboard.putNumber("Encoder", Counter);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    // This method will be called once per scheduler run
  }
}
