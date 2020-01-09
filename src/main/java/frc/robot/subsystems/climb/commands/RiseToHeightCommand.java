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
import frc.robot.subsystems.climb.Climb;

/**
 * An example command that uses an example subsystem.
 */
public class RiseToHeightCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climb m_subsystem;
    private double targetHeight;
    private double targetAngle;
    private double currentHeight;
    private double currentAngleError;
    private double leftTargetHeight;
    private double rightTargetHeight;

    /**
     * Creates a new RiseToHeightCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RiseToHeightCommand(Climb subsystem, double targetHeight) {
        m_subsystem = subsystem;
        this.targetHeight = targetHeight;
        this.targetAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    /**
     * @param subsystem    the subsystem
     * @param targetHeight the desired height for the mechanism
     * @param targetAngle  the desired angle
     */
    public RiseToHeightCommand(Climb subsystem, double targetHeight, double targetAngle) {
        this.m_subsystem = subsystem;
        this.targetHeight = targetHeight;
        this.targetAngle = targetAngle;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.releaseStopper();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Update the target height of each side
        leftTargetHeight = targetHeight - m_subsystem.getLeftHeight();
        rightTargetHeight = targetHeight - m_subsystem.getRightHeight();

        //Calculate the error angle and the current height
        currentAngleError = targetAngle - Robot.navx.getRoll();
        currentHeight = (m_subsystem.getLeftHeight() + m_subsystem.getRightHeight()) / 2;

        //If the elevator reaches the target height engage the mechanical stopper to stop it from going up
        if (Math.abs(targetHeight - currentHeight) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE) {
            m_subsystem.engageStopper();
        }

        //Fix the heights according to the angle of the robot
        if (currentAngleError > 0) {
            rightTargetHeight -= Constants.ROBOT_WIDTH * Math.tan(Math.toRadians(currentAngleError));
        } else {
            leftTargetHeight -= Constants.ROBOT_WIDTH * Math.tan(Math.toRadians(Math.abs(currentAngleError)));
        }

        m_subsystem.setLeftHeight(leftTargetHeight);
        m_subsystem.setRightHeight(rightTargetHeight);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.engageStopper();
    }


    /**
     * Returns true when the command should end.
     * @return if the left and the right side are on the correct height and the angle is correct
     */
    @Override
    public boolean isFinished() {
        return Math.abs(leftTargetHeight - m_subsystem.getLeftHeight()) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE && Math.abs(rightTargetHeight - m_subsystem.getRightHeight()) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE
                && Math.abs(currentAngleError) < Constants.Climb.CLIMB_ANGLE_TOLERANCE;
    }
}
