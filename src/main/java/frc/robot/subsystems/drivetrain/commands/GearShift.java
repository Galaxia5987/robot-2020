/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Drivetrain.*;

/**
 *
 */
public class GearShift extends CommandBase {
  private Drivetrain.shiftModes shiftmode;
  private final Drivetrain drivetrain;

  /**
   *
   *
   *
   */
  public GearShift(Drivetrain drivetrain, Drivetrain.shiftModes shiftmode) {
    this.shiftmode = shiftmode;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    return drivetrain.getCooldown() > SHIFTER_COOLDOWN;
  }
}
