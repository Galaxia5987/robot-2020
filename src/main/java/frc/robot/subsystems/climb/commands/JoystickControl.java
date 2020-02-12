/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.climb.Climber;

/**
 * This command would allow the driver to modify the robot's angle manually
 * with the value of the left xbox's joystick.
 */
public class JoystickControl extends CommandBase {
    private final Climber climber;
    private final boolean manual;
    private boolean controlEachSide;
    private double rightInput, leftInput;

    /**
     * Creates a new joystick control command.
     *
     * @param climber The subsystem used by this command.
     */
    public JoystickControl(Climber climber, boolean controlEachSide, boolean manual) {
        this.climber = climber;
        this.controlEachSide = controlEachSide;
        this.manual = manual;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
        climber.changePIDFSlot(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (controlEachSide) {
            leftInput = OI.getXboxLY() * Constants.Climber.MODIFY_JOYSTICK_RATE.get();
            rightInput = OI.getXboxRY() * Constants.Climber.MODIFY_JOYSTICK_RATE.get();
        }
        else {
            leftInput = OI.getXboxLY() * Constants.Climber.MODIFY_JOYSTICK_RATE.get();
            rightInput = -leftInput;
        }
        if(manual) {
            climber.setLeftPower(leftInput);
            climber.setRightPower(rightInput);
        }
        else {
            climber.setLeftHeight(leftInput + climber.getLeftHeight());
            climber.setRightHeight(rightInput + climber.getRightHeight());
        }
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
        return false;
    }


}
