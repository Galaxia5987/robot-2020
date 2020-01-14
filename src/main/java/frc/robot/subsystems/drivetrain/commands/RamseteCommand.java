/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.drivetrain;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
public class RamseteCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private static final RamseteController m_follower = Constants.Drivetrain.follower;
    private final SimpleMotorFeedforward m_feedforward = Constants.Drivetrain.feedforward;
    private final DifferentialDriveKinematics m_kinematics = Constants.Drivetrain.kDriveKinematics;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    public RamseteCommand(Trajectory trajectory) {
        m_trajectory = trajectory;
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(drivetrain.getPose(), m_trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward =
                m_feedforward.calculate(leftSpeedSetpoint,
                        (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
                m_feedforward.calculate(rightSpeedSetpoint,
                        (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        drivetrain.setVelocityAndFeedForward(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}
