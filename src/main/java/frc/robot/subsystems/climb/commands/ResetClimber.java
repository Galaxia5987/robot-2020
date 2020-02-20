/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climber;

/**
 * This command would reset the rods height to 0 (their initial state).
 */
public class ResetClimber extends CommandBase {
    private final Climber climber;
    private final Timer timer = new Timer();
    /**
     * Creates a new reset climber command.
     *
     * @param climber The subsystem used by this command.
     */
    public ResetClimber(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
        climber.changePIDFSlot(1);
        climber.setLeftPower(-0.3);
        climber.setRightPower(-0.3);
        timer.reset();
        timer.start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.setLeftPower(0);
        climber.setRightPower(0);
        climber.engageStopper();
        if(!interrupted) {
            climber.resetEncoders();
        }
    }


    /**
     * This command is designed to work only while a button is held,
     * so it would terminate itself if the button is released so there is no need for isFinished.
     *
     * @return false
     */
    @Override
    public boolean isFinished() {
        return timer.get() > 4.; //TODO: find better isFinished when time is due.
    }


}
