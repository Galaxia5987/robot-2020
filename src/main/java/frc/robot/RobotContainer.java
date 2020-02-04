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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.ManualControl;
import frc.robot.subsystems.color_wheel.commands.PositionControl;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.intake.commands.OuttakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.utilities.ButtonCombination;
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
    public static final Climber climber = new Climber();
    public static final ColorWheel colorWheel = new ColorWheel();
    public static final Turret turret = new Turret();
    public static final Conveyor conveyor = new Conveyor();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Intake intake = new Intake();
    public static final Shooter shooter = new Shooter();
    private final Command m_autoCommand = null;
    public static JoystickButton a = new JoystickButton(OI.xbox, 1);
    public static JoystickButton b = new JoystickButton(OI.xbox, 2);
    public static JoystickButton y = new JoystickButton(OI.xbox, 3);
    public static JoystickButton x = new JoystickButton(OI.xbox, 4);
    public static JoystickButton select = new JoystickButton(OI.xbox, 7);
    public static JoystickButton start = new JoystickButton(OI.xbox, 8);
    public static JoystickButton rb = new JoystickButton(OI.xbox, 5);
    public static JoystickButton lb = new JoystickButton(OI.xbox, 6);
    public static ButtonCombination select_start = new ButtonCombination(OI.xbox, 7, 8);

    public RobotContainer() {
        configureButtonBindings();
        colorWheel.setDefaultCommand(new ManualControl(colorWheel));
        turret.setDefaultCommand(new JoystickTurret(turret));
        if (Robot.debug) {
            startFireLog();
        }
    }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private void configureButtonBindings() {
        a.whenPressed(new IntakeBalls(intake, 0.4));
        x.whileHeld(new OuttakeBalls(conveyor, intake, 0.4));
        b.whenPressed(new SpeedUp(shooter));
        y.whenPressed(new FeedTurret(conveyor));
        select.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        rb.whenPressed(new RotationControl(colorWheel));
        lb.whenPressed(new PositionControl(colorWheel));
        select_start.whenHeld(new SequentialCommandGroup(
                new WaitCommand(2),
                new RunCommand(() -> Robot.shootingManualMode = true)
        )); //If both buttons are held without being released the manualMode will be enabled.
        start.whenPressed(() -> Robot.shootingManualMode = false); //Pressing start disables the manual mode for shooting.
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
