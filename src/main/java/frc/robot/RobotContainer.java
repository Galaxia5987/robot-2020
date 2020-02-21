/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.commands.PIDClimbAndBalance;
import frc.robot.subsystems.climb.commands.ReleaseRods;
import frc.robot.subsystems.climb.commands.ResetClimber;
import frc.robot.subsystems.climb.commands.SimpleClimb;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.ManualControl;
import frc.robot.subsystems.color_wheel.commands.PositionControl;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.GearShift;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commandgroups.OuttakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootAtVelocity;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.VisionModule;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    private final VisionModule visionModule = new VisionModule();
    private final CustomDashboard customDashboard = new CustomDashboard();
    private final Drivetrain drivetrain = new Drivetrain();
    private final ColorWheel colorWheel = new ColorWheel();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor(intake);
    public static final Climber climber = new Climber();
    public static final Turret turret = new Turret();
    private final Command m_autoCommand = null;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        navx.reset();
        configureDefaultCommands();
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }
    }

    /**
     * Defines the default command of each mechanism on the robot.
     */
    private void configureDefaultCommands(){
    }
    /**
     * Configures all of the button usages on the robot.
     */
    private void configureButtonBindings() {
        OI.lb.whileHeld(new JoystickDrive(drivetrain, ()-> OI.getXboxLY()*-0.3, ()-> OI.getXboxRY()*-0.3));
        OI.back.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        OI.a.whenPressed(new ReleaseRods(climber));
        OI.b.whenPressed(new ResetClimber(climber));
        OI.x.toggleWhenPressed(new PIDClimbAndBalance(climber));
        OI.y.whenPressed(new SimpleClimb(climber));
        OI.start.whenPressed(new RunCommand(climber::engageStopper));
        OI.rs.whenPressed(new RunCommand(climber::releaseStopper));

    }

    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
        new ValueTuner().start();
    }

    /**
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {
        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

}