/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climber;

/**
 * This command would allow the driver to modify the robot's angle manually
 * with the value of the left xbox's joystick.
 */
public class ReleaseRods extends CommandBase {
    private final Climber climber;
    private double currentHeight;
    private double setpointHeight;



    /**
     * Creates a new joystick control command.
     *
     * @param climber The subsystem used by this command.
     */
    public ReleaseRods(Climber climber, double setpointHeight) {
        this.climber = climber;
        this.setpointHeight = setpointHeight;
        climber.changePIDFSlot(1);
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentHeight = (climber.getLeftHeight() + climber.getRightHeight()) / 2;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }


    /**
     * This command is designed to work only while a button is held,
     * so it would terminate itself if the button is released so there is no need for isFinished.
     *
     * @return false
     */
    @Override
    public final boolean isFinished() {
        return Math.abs(setpointHeight - currentHeight) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
    }


}
