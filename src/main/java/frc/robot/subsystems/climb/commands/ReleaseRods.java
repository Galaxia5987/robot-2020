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
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
import static frc.robot.Constants.Climber.CLIMB_HEIGHT;

/**
 * This command would allow the driver to modify the robot's angle manually
 * with the value of the left xbox's joystick.
 */
public class ReleaseRods extends CommandBase {
    private final Climber climber;
    private double setpointHeight;

    /**
     * Creates a new joystick control command.
     *
     * @param climber The subsystem used by this command.
     */
    public ReleaseRods(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.setpointHeight = CLIMB_HEIGHT;
        climber.releaseStopper();
        climber.changePIDFSlot(1);
        climber.setLeftHeight(setpointHeight);
        climber.setRightHeight(setpointHeight);
    }

    @Override
    public void execute() {
        FireLog.log("climbLeftHeight", climber.getLeftHeight());
        FireLog.log("climbRightHeight", climber.getRightHeight());
        FireLog.log("climberSetpoint", setpointHeight);
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
        boolean isLeftOnSetpoint = climber.getLeftHeight() > setpointHeight - ALLOWED_HEIGHT_TOLERANCE;
        boolean isRightOnSetpoint = climber.getRightHeight() > setpointHeight - ALLOWED_HEIGHT_TOLERANCE;

        return isLeftOnSetpoint && isRightOnSetpoint;
    }


}
