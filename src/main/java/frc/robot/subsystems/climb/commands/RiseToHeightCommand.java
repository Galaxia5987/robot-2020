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
import frc.robot.subsystems.climb.Climber;

/**
 * An example command that uses an example subsystem.
 */
public class RiseToHeightCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber climber;
    private double targetHeight;
    private double targetAngle;
    private double currentHeight;
    private double currentAngleError;
    private double leftTargetHeight;
    private double rightTargetHeight;

    /**
     * Creates a new RiseToHeightCommand.
     *
     * @param climber The subsystem used by this command.
     */
    public RiseToHeightCommand(Climber climber, double targetHeight) {
        this.climber = climber;
        this.targetHeight = targetHeight;
        this.targetAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    /**
     * @param subsystem    the subsystem
     * @param targetHeight the desired height for the mechanism
     * @param targetAngle  the desired angle
     */
    public RiseToHeightCommand(Climber subsystem, double targetHeight, double targetAngle) {
        this.climber = subsystem;
        this.targetHeight = targetHeight;
        this.targetAngle = targetAngle;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseStopper();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Update the target height of each side
        leftTargetHeight = targetHeight - climber.getLeftHeight();
        rightTargetHeight = targetHeight - climber.getRightHeight();

        //Calculate the error angle and the current height
        currentAngleError = targetAngle - Robot.navx.getRoll();
        currentHeight = (climber.getLeftHeight() + climber.getRightHeight()) / 2;

        //If the elevator reaches the target height engage the mechanical stopper to stop it from going up
        if (Math.abs(targetHeight - currentHeight) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE) {
            climber.engageStopper();
        }

        double difference = Constants.ROBOT_WIDTH * Math.tan(Math.toRadians(Math.abs(currentAngleError)));
        //Fix the heights according to the angle of the robot
        if (currentAngleError > 0) {
            double[] heights = normalizeHeights(difference, rightTargetHeight, leftTargetHeight, 0, Constants.Climb.CLIMB_HEIGHT);
            rightTargetHeight = heights[0];
            leftTargetHeight = heights[1];
        } else {
            double[] heights = normalizeHeights(difference, leftTargetHeight, rightTargetHeight, 0, Constants.Climb.CLIMB_HEIGHT);
            rightTargetHeight = heights[0];
            leftTargetHeight = heights[1];
        }

        climber.setLeftHeight(leftTargetHeight);
        climber.setRightHeight(rightTargetHeight);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.engageStopper();
    }


    /**
     * Returns true when the command should end.
     * @return whether the left and the right side are on at their setpoints and the angle is correct (=the robot base is parallel to the ground)
     */
    @Override
    public boolean isFinished() {
        return Math.abs(leftTargetHeight - climber.getLeftHeight()) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE && Math.abs(rightTargetHeight - climber.getRightHeight()) < Constants.Climb.CLIMB_HEIGHT_TOLERANCE
                && Math.abs(currentAngleError) < Constants.Climb.CLIMB_ANGLE_TOLERANCE;
    }

    /**
     * This method would take 2 values and increase the difference between them,
     * without exceeding min and max limits
     * @param difference the difference
     * @param firstHeight the first height
     * @param secondHeight the second height
     * @param minLimit the minimum limit
     * @param maxLimit the maximum limit
     * @return the modified heights
     */
    private double[] normalizeHeights(double difference, double firstHeight, double secondHeight, double minLimit, double maxLimit) {
        if (firstHeight - difference >= minLimit) {
            firstHeight -= difference;
        } else {
            firstHeight -= difference;
            difference = Math.abs(Math.abs(firstHeight) - minLimit);
            firstHeight = minLimit;
            if (secondHeight + difference <= maxLimit) {
                secondHeight += difference;
            } else {
                secondHeight = maxLimit;
            }
        }
        return new double[]{firstHeight, secondHeight};
    }

}
