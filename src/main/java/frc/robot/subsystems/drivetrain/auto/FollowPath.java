/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This command handles trajectory-following.
 * A modified fork of {@link edu.wpi.first.wpilibj2.command.RamseteCommand}
 */
public class FollowPath extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;

    private static final RamseteController follower = new RamseteController(Constants.Autonomous.kBeta, Constants.Autonomous.kZeta);
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Autonomous.Ks, Constants.Autonomous.Kv, Constants.Autonomous.Ka);
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private final Drivetrain drivetrain;

    public FollowPath(Drivetrain drivetrain, Trajectory trajectory) {
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        prevTime = 0;
        var initialState = trajectory.sample(0);

        prevSpeeds = kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        var targetWheelSpeeds = kinematics.toWheelSpeeds(
                follower.calculate(drivetrain.getPose(), trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward =
                feedforward.calculate(leftSpeedSetpoint,
                        (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
                feedforward.calculate(rightSpeedSetpoint,
                        (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

        drivetrain.setVelocityAndFeedForward(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

        prevTime = curTime;
        prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
    }
}
