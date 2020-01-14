/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climber;

/**
 * An example command that uses an example subsystem.
 */
public class JoystickControlCommand extends CommandBase {
    private final Climber climber;


    /**
     * Creates a new RiseToHeightCommand.
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
        climber.setLeftSpeed(RobotContainer.getLeftXboxX()*Constants.Climb.MODIFY_JOYSTICK_SPEED);
        climber.setRightSpeed(-RobotContainer.getLeftXboxX()*Constants.Climb.MODIFY_JOYSTICK_SPEED);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.engageStopper();
    }


    /**
     * Returns true when the command should end.
     *
     * @return whether the left and the right side are on at their setpoints and the angle is correct (=the robot base is parallel to the ground)
     */
    @Override
    public boolean isFinished() {
        return false;
    }


}
