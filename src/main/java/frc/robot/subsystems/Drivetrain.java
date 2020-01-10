/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UtilityFunctions;
import frc.robot.utilities.FalconConfiguration;

public class Drivetrain extends SubsystemBase {


  private final TalonFX rightMaster = new TalonFX(Ports.Drivetrain.RIGHT_MASTER);
  private final TalonFX rightSlave = new TalonFX(Ports.Drivetrain.RIGHT_SLAVE);
  private final TalonFX leftMaster = new TalonFX(Ports.Drivetrain.LEFT_MASTER);
  private final TalonFX leftSlave = new TalonFX(Ports.Drivetrain.LEFT_SLAVE);
  private FalconConfiguration configurations = new FalconConfiguration();
  private double[] pidSet = {Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD, Constants.Drivetrain.KF};

  /**
   * Creates a new ExampleSubsystem.
   */
  public Drivetrain() {
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);
    configurations.setNeutralMode(NeutralMode.Coast);
    configurations.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    configurations.setEnableVoltageCompensation(true);
    configurations.setPidSet(pidSet);
    UtilityFunctions.configAllFalcons(configurations, rightMaster, rightSlave, leftMaster, leftSlave);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
