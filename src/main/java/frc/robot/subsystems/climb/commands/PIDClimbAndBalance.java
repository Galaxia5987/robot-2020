/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climber;

/**
 * An example command that uses an example subsystem.
 */
public class PIDClimbAndBalance extends CommandBase {
    private final Climber climber;
    private MiniPID deltaPID = new MiniPID(Constants.Climber.DELTA_PID[0], Constants.Climber.DELTA_PID[1], Constants.Climber.DELTA_PID[2]);
    private double setpointHeightFromGround;
    private double setpointAngle;
    private double delta = 0;
    private double currentAngleError;
    private double leftSetpointHeight;
    private double rightSetpointHeight;

    /**
     * Creates a new rise to height command.
     *
     * @param climber The subsystem used by this command.
     */
    public PIDClimbAndBalance(Climber climber) {
        this.climber = climber;
        this.setpointHeightFromGround = (climber.getLeftHeight() + climber.getRightHeight()) / 2;
        this.setpointAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    /**
     * Creates a new rise to height command.
     *
     * @param climber The subsystem used by this command.
     */
    public PIDClimbAndBalance(Climber climber, double setpointHeightFromGround) {
        this.climber = climber;
        this.setpointHeightFromGround = setpointHeightFromGround;
        this.setpointAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    /**
     * @param subsystem                the subsystem
     * @param setpointHeightFromGround the desired height for the mechanism
     * @param setpointAngle            the desired angle
     */
    public PIDClimbAndBalance(Climber subsystem, double setpointHeightFromGround, double setpointAngle) {
        this.climber = subsystem;
        this.setpointHeightFromGround = setpointHeightFromGround;
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
        leftSetpointHeight = setpointHeightFromGround;
        rightSetpointHeight = setpointHeightFromGround;

        //Calculate the error angle and the current height
        currentAngleError = setpointAngle - RobotContainer.navx.getRoll();

        delta += deltaPID.getOutput((RobotContainer.navx.getRoll() - setpointAngle), 0);
        delta = climber.normalizeDelta(delta);

        leftSetpointHeight += delta * 0.5;
        rightSetpointHeight -= delta * 0.5;

        if (!climber.isStopperEngaged()) {
            climber.setLeftHeight(leftSetpointHeight);
            climber.setRightHeight(rightSetpointHeight);
        } else {
            climber.releaseStopper();
        }
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
        if (difference > 0) {
            firstHeight += difference;
            if (firstHeight >= maxLimit) {
                difference = Math.abs(maxLimit - firstHeight);
                firstHeight = maxLimit;
                if (secondHeight - difference <= minLimit) {
                    secondHeight = minLimit;
                } else {
                    secondHeight -= difference;
                }
            }
        } else {
            secondHeight += difference;
            if (secondHeight >= maxLimit) {
                difference = Math.abs(maxLimit - secondHeight);
                secondHeight = maxLimit;
                if (firstHeight - difference <= minLimit) {
                    firstHeight = minLimit;
                } else {
                    firstHeight -= difference;
                }
            }
        }
        return new double[]{firstHeight, secondHeight};
    }
}