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
public class RiseToHeight extends CommandBase {
    private final Climber climber;
    private boolean startBalancing = false;
    private double setpointHeight;
    private double setpointAngle;
    private double currentHeight;
    private double currentAngleError;
    private double leftSetpointHeight;
    private double rightSetpointHeight;

    /**
     * Creates a new rise to height command.
     *
     * @param climber The subsystem used by this command.
     */
    public RiseToHeight(Climber climber, double setpointHeight) {
        this.climber = climber;
        this.setpointHeight = setpointHeight;
        this.setpointAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    /**
     * @param subsystem      the subsystem
     * @param setpointHeight the desired height for the mechanism
     * @param setpointAngle  the desired angle
     */
    public RiseToHeight(Climber subsystem, double setpointHeight, double setpointAngle) {
        this.climber = subsystem;
        this.setpointHeight = setpointHeight;
        this.setpointAngle = setpointAngle;
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
        leftSetpointHeight = setpointHeight - climber.getLeftHeight();
        rightSetpointHeight = setpointHeight - climber.getRightHeight();

        //Calculate the error angle and the current height
        currentAngleError = setpointAngle - Robot.navx.getRoll();
        currentHeight = (climber.getLeftHeight() + climber.getRightHeight()) / 2;

        //If the elevator reaches the target height engage the mechanical stopper to stop it from going up
        if (Math.abs(setpointHeight - currentHeight) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE) {
            startBalancing = true;
        }

        if (startBalancing && !climber.isStopperEngaged()) {
            double targetDifference = Constants.ROBOT_WIDTH * Math.tan(Math.toRadians(Math.abs(currentAngleError)));
            //Fix the heights according to the angle of the robot
            if (currentAngleError > 0) {
                double[] heights = normalizeHeights(targetDifference, rightSetpointHeight, leftSetpointHeight, 0, Constants.Climber.HEIGHT);
                rightSetpointHeight = heights[0];
                leftSetpointHeight = heights[1];
            } else {
                double[] heights = normalizeHeights(targetDifference, leftSetpointHeight, rightSetpointHeight, 0, Constants.Climber.HEIGHT);
                rightSetpointHeight = heights[1];
                leftSetpointHeight = heights[0];
            }

            climber.setLeftHeight(leftSetpointHeight);
            climber.setRightHeight(rightSetpointHeight);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.engageStopper();
        startBalancing = false;
    }


    /**
     * Returns true when the command should end.
     *
     * @return whether the left and the right side are on at their setpoints and the angle is correct (=the robot base is parallel to the ground)
     */
    @Override
    public boolean isFinished() {
        boolean isLeftOnSetpoint = Math.abs(leftSetpointHeight - climber.getLeftHeight()) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
        boolean isRightOnSetpoint = Math.abs(rightSetpointHeight - climber.getRightHeight()) < Constants.Climber.ALLOWED_HEIGHT_TOLERANCE;
        boolean isAngleOnSetpoint = Math.abs(currentAngleError) < Constants.Climber.ALLOWED_ANGLE_TOLERANCE;
        return isLeftOnSetpoint && isRightOnSetpoint && isAngleOnSetpoint;
    }

    /**
     * This method would take 2 values and increase the difference between them,
     * without exceeding min and max limits
     *
     * @param difference   the difference
     * @param firstHeight  the first height
     * @param secondHeight the second height
     * @param minLimit     the minimum limit
     * @param maxLimit     the maximum limit
     * @return the modified heights
     */
    public double[] normalizeHeights(double difference, double firstHeight, double secondHeight, double minLimit, double maxLimit) {
        firstHeight -= difference;
        if (firstHeight <= minLimit) {
            difference = Math.abs(Math.abs(firstHeight) - minLimit);
            firstHeight = minLimit;
            if (secondHeight + difference >= maxLimit) {
                secondHeight = maxLimit;
            } else {
                secondHeight += difference;

            }
        }
        return new double[]{firstHeight, secondHeight};
    }
}
