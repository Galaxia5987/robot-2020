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
import frc.robot.valuetuner.WebConstant;

/**
 * An example command that uses an example subsystem.
 */
public class PIDClimbAndBalance extends CommandBase {
    private final Climber climber;
    private MiniPID deltaPID = new MiniPID(Constants.Climber.DELTA_PID[0], Constants.Climber.DELTA_PID[1], Constants.Climber.DELTA_PID[2]);
    private double setpointHeight;
    private double setpointAngle;
    private double delta = 0;
    private double currentAngleError;
    private double leftSetpointHeight;
    private double rightSetpointHeight;

    private WebConstant p = new WebConstant("delta p", 0);
    private WebConstant i = new WebConstant("delta i", 0);
    private WebConstant d = new WebConstant("delta d", 0);

    /**
     * Creates a new rise to height command.
     *
     * @param climber The subsystem used by this command.
     */
    public PIDClimbAndBalance(Climber climber) {
        this.climber = climber;
        this.setpointHeight = Constants.Climber.HEIGHT_TARGET;
        this.setpointAngle = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    /**
     * Creates a new rise to height command.
     *
     * @param climber The subsystem used by this command.
     */
    public PIDClimbAndBalance(Climber climber, double setpointHeight) {
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
    public PIDClimbAndBalance(Climber subsystem, double setpointHeight, double setpointAngle) {
        this.climber = subsystem;
        this.setpointHeight = setpointHeight;
        this.setpointAngle = setpointAngle;
        addRequirements(subsystem);
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
        updatePID();
        //Update the target height of each side
        leftSetpointHeight = setpointHeight;
        rightSetpointHeight = setpointHeight;

        //Calculate the error angle and the current height
        currentAngleError = setpointAngle + RobotContainer.navx.getRoll();

        delta = deltaPID.getOutput(currentAngleError, 0);
        delta = climber.normalizeDelta(delta);
        if (Math.abs(delta) <= Constants.Climber.MIN_DELTA) delta = 0;

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
        climber.setLeftHeight(climber.getLeftHeight());
        climber.setRightHeight(climber.getRightHeight());
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

    private void updatePID(){
        deltaPID.setP(p.get());
        deltaPID.setI(i.get());
        deltaPID.setD(d.get());
        deltaPID.setF(0);
    }
}
