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
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.ManualControl;
import frc.robot.subsystems.color_wheel.commands.PositionControl;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.intake.commands.OuttakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.utilities.StickButton;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;

import static frc.robot.OI.rightStick;
import static frc.robot.OI.xbox;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
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
        configureDefaultCommands();
        configureButtonBindings();
        if (Robot.debug) {
            startFireLog();
        }
    }

    /**
     * Defines the default command of each mechanism on the robot.
     */
    private void configureDefaultCommands(){
        colorWheel.setDefaultCommand(new ManualControl(colorWheel));
        turret.setDefaultCommand(new JoystickTurret(turret));
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain));
    }
    /**
     * Configures all of the button usages on the robot.
     */
    private void configureButtonBindings() {
        OI.a.whenPressed(new IntakeBalls(intake, 0.35));
        OI.x.whileHeld(new OuttakeBalls(conveyor, intake, 0.2, 0.5));
        OI.b.whenPressed(new SpeedUp(shooter));
        OI.y.whenPressed(new FeedTurret(conveyor));
        OI.select.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        OI.rb.whenPressed(new RotationControl(colorWheel));
        OI.lb.whenPressed(new PositionControl(colorWheel));
        OI.select_start.whenHeld(new SequentialCommandGroup(
                new WaitCommand(2),
                new RunCommand(() -> Robot.shootingManualMode = true)
        )); //If both buttons are held without being released the manualMode will be enabled.
        OI.start.whenPressed(() -> Robot.shootingManualMode = false); //Pressing start disables the manual mode for shooting.
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