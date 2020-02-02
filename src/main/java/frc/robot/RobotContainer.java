/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    public static Joystick rightJoystick = new Joystick(0);
    public static Joystick leftJoystick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static JoystickButton a = new JoystickButton(xbox, 1);
    public static JoystickButton b = new JoystickButton(xbox, 2);
    public static JoystickButton y = new JoystickButton(xbox, 3);
    public static JoystickButton x = new JoystickButton(xbox, 4);
    public static JoystickButton select = new JoystickButton(xbox, 5); // TODO: check the actual number
    public static JoystickButton cancel = new JoystickButton(xbox, 6);
    public static JoystickButton rb = new JoystickButton(xbox, 7);
    public static JoystickButton lb = new JoystickButton(xbox, 8);
    public static final int XboxLeftXStick = 0;
    public static final int XboxLeftYStick = 1;
    public static final int XboxRightYStick = 5;
    public static final int XboxRightXStick = 4;

    public RobotContainer() {
        configureButtonBindings();
        colorWheel.setDefaultCommand(new ManualControl(colorWheel, RobotContainer::getRightXboxX));
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
        cancel.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        rb.whenPressed(new RotationControl(colorWheel));
        lb.whenPressed(new PositionControl(colorWheel));

    }

    public static double getLeftXboxX() {
        return xbox.getRawAxis(XboxLeftXStick);
    }

    public static double getLeftXboxY() {
        return xbox.getRawAxis(XboxLeftYStick);
    }

    public static double getRightXboxY() {
        return xbox.getRawAxis(XboxRightYStick);
    }

    public static double getRightXboxX() {
        return xbox.getRawAxis(XboxRightXStick);
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
