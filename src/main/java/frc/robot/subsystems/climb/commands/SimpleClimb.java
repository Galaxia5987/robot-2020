/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climber;

import static frc.robot.Constants.Climber.SIMPLE_CLIMB_HEIGHT;

public class SimpleClimb extends CommandBase {
    private final Climber climber;

    public SimpleClimb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
        climber.changePIDFSlot(0);
        climber.setLeftHeight(SIMPLE_CLIMB_HEIGHT);
        climber.setRightHeight(SIMPLE_CLIMB_HEIGHT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.engageStopper();
    }


    /**
     * This command is designed to work only while a button is held,
     * so it would terminate itself if the button is released so there is no need for isFinished.
     *
     * @return false
     */
    @Override
    public boolean isFinished() {
        boolean leftSideInPosition = Math.abs(SIMPLE_CLIMB_HEIGHT - climber.getLeftHeight()) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
        boolean rightSideInPosition = Math.abs(SIMPLE_CLIMB_HEIGHT - climber.getRightHeight()) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
        return leftSideInPosition && rightSideInPosition;
    }


}
