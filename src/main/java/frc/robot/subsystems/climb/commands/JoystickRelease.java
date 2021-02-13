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

import java.util.function.Supplier;

/**
 * This command would allow the driver to modify the robot's angle manually
 * with the value of the left xbox's joystick.
 */
public class JoystickRelease extends CommandBase {
    private final Climber climber;
    private double setpoint;
    private Supplier<Double> joystickInput = OI::getXboxRY;

    /**
     * Creates a new joystick control command.
     *
     * @param climber The subsystem used by this command.
     */
    public JoystickRelease(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
        climber.changePIDFSlot(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        setpoint = -joystickInput.get() * Constants.Climber.MODIFY_JOYSTICK_RATE;
        if (Math.abs((climber.getLeftHeight() + climber.getRightHeight()) / 2 - setpoint) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE) {
            setpoint = 0;
        }

        climber.setLeftHeight(setpoint + climber.getLeftHeight());
        climber.setRightHeight(setpoint + climber.getRightHeight());

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
    public boolean isFinished() {
        return false;
    }


}
