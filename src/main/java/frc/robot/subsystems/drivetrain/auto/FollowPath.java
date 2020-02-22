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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.autonomous.Path;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utilities.Utils;
import org.ghrobotics.lib.debug.FalconDashboard;
import org.techfire225.webapp.FireLog;

/**
 * This command handles trajectory-following.
 * A modified fork of {@link edu.wpi.first.wpilibj2.command.RamseteCommand}
 */
public class FollowPath extends CommandBase {
    private final Timer timer = new Timer();
    private boolean resetDrivetrain = false;
    private Path path;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;

    private static final RamseteController follower = new RamseteController(Constants.Autonomous.kBeta, Constants.Autonomous.kZeta);
    private static final SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.leftkS, Constants.Autonomous.leftkV, Constants.Autonomous.leftkA);
    private static final SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.rightkS, Constants.Autonomous.rightkV, Constants.Autonomous.rightkA);
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private final Drivetrain drivetrain;
    private Trajectory trajectory;

    public FollowPath(Drivetrain drivetrain, Trajectory trajectory, boolean resetDrivetrain) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        this.resetDrivetrain = resetDrivetrain;
    }

    public FollowPath(Drivetrain drivetrain, Path path) {
        addRequirements(drivetrain);
        this.path = path;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        if(trajectory == null) {
            if (!path.hasTrajectory()) {
                path.generate(drivetrain.getPose());
            }

            this.trajectory = path.getTrajectory();
        }

        if(resetDrivetrain)
            drivetrain.setPose(trajectory.getInitialPose());
        System.out.println(String.format("Initial pose: %s, %s, %s", trajectory.getInitialPose().getTranslation().getX(), trajectory.getInitialPose().getTranslation().getY(), trajectory.getInitialPose().getRotation().getDegrees()));
        FalconDashboard.INSTANCE.setFollowingPath(true);
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

        Trajectory.State state = trajectory.sample(curTime);

        var targetWheelSpeeds = kinematics.toWheelSpeeds(
                follower.calculate(drivetrain.getPose(), state)
        );

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        FireLog.log("autoLeftSetpoint", Math.abs(leftSpeedSetpoint));
        FireLog.log("autoRightSetpoint", Math.abs(rightSpeedSetpoint));

        double leftFeedforward =
                leftfeedforward.calculate(leftSpeedSetpoint,
                        (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
                rightfeedforward.calculate(rightSpeedSetpoint,
                        (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

        drivetrain.setVelocityAndFeedForward(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward / 12, rightFeedforward / 12);

        FireLog.log("autoLeftTarget", leftSpeedSetpoint);
        FireLog.log("autoLeftVelocity", drivetrain.getLeftVelocity());

        FalconDashboard.INSTANCE.setPathHeading(state.poseMeters.getRotation().getRadians());
        FalconDashboard.INSTANCE.setPathX(Units.metersToFeet(state.poseMeters.getTranslation().getX()));
        FalconDashboard.INSTANCE.setPathY(Units.metersToFeet(state.poseMeters.getTranslation().getY()));

        prevTime = curTime;
        prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        FalconDashboard.INSTANCE.setFollowingPath(false);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
    }
}
