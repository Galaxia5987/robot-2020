/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 *
 */
public class DriveForward extends CommandBase {
    private final Drivetrain drivetrain;
    private double leftPower, rightPower;

    /**
     *
     */
    public DriveForward(Drivetrain drivetrain, double leftPower, double rightPower) {
        this.drivetrain = drivetrain;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.setPower(leftPower, rightPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double leftJoystick = Math.abs(OI.getLeftStickForward());
        double rightJoystick = Math.abs(OI.getRightStickForward());
        return leftJoystick > Constants.Drivetrain.JOYSTICK_END_THRESHOLD ||
                rightJoystick > Constants.Drivetrain.JOYSTICK_END_THRESHOLD;
    }
}
