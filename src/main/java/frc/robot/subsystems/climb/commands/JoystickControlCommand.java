/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climber;

/**
 * This command would allow the driver to modify the robot's angel manually
 * with the value of the left xbox's joystick.
 */
public class JoystickControlCommand extends CommandBase {
    private final Climber climber;


    /**
     * Creates a new JoystickControlCommand.
     *
     * @param climber The subsystem used by this command.
     */
    public JoystickControlCommand(Climber climber) {
        this.climber = climber;
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
        climber.setLeftHeight(RobotContainer.getLeftXboxX()*Constants.Climb.MODIFY_JOYSTICK_RATE + climber.getLeftHeight());
        climber.setRightHeight(-RobotContainer.getLeftXboxX()*Constants.Climb.MODIFY_JOYSTICK_RATE + climber.getRightHeight());
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.engageStopper();
    }


    @Override
    public boolean isFinished() {
        return false;
    }


}
