/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color BlueTarget = ColorMatch.makeColor(Constants.ColorWheel.BLUE_RGB[0], Constants.ColorWheel.BLUE_RGB[1], Constants.ColorWheel.BLUE_RGB[2]);
  private final Color GreenTarget = ColorMatch.makeColor(Constants.ColorWheel.GREEN_RGB[0], Constants.ColorWheel.GREEN_RGB[1], Constants.ColorWheel.GREEN_RGB[2]);
  private final Color RedTarget = ColorMatch.makeColor(Constants.ColorWheel.RED_RGB[0], Constants.ColorWheel.RED_RGB[1], Constants.ColorWheel.RED_RGB[2]);
  private final Color YellowTarget = ColorMatch.makeColor(Constants.ColorWheel.YELLOW_RGB[0], Constants.ColorWheel.YELLOW_RGB[1], Constants.ColorWheel.YELLOW_RGB[2]);
  /**
   * Creates a new ExampleSubsystem.
   */
  public ColorWheel() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
