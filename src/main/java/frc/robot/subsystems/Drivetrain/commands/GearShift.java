/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Drivetrain.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.drivetrain;

/**
 *
 */
public class GearShift extends CommandBase {
  private Drivetrain.shiftModes shiftmode;

  /**
   *
   *
   *
   */
  public GearShift(Drivetrain.shiftModes shiftmode) {
    this.shiftmode = shiftmode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.startCooldown();
    drivetrain.shiftGear(shiftmode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetCooldown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getCooldown() > Constants.Drivetrain.SHIFTER_COOLDOWN;
  }
}
