/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.commands.CalculatedClimbAndBalance;
import frc.robot.subsystems.climb.commands.JoystickControl;
import frc.robot.subsystems.climb.commands.ReleaseRods;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.utilities.TrajectoryLoader;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.*;
import frc.robot.utilities.StickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final Climber climber = new Climber();
    public final Turret turret = new Turret();
    private final Command m_autoCommand = null;
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    public static XboxController xbox = new XboxController(2);
    public static JoystickButton a = new JoystickButton(xbox, 1);
    public static JoystickButton b = new JoystickButton(xbox, 2);
    public static JoystickButton y = new JoystickButton(xbox, 3);
    public static final int XboxLeftXStick = 0;
    public static final int XboxLeftYStick = 1;
    public static final int XboxRightYStick = 5;
    private final JoystickButton rs = new JoystickButton(xbox, 10);
    private final StickButton rightY = new StickButton(xbox, 5, 0.1);

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        a.whileHeld(new JoystickControl(climber, false));
        b.whenPressed(new ReleaseRods(climber, 1.5));
        y.whenPressed(new CalculatedClimbAndBalance(climber, 1));
        rightY.whileHeld(new JoystickTurret(turret));
        rs.whenPressed(new InstantCommand(turret::resetOffset));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }

    public static double getLeftXboxX() {
        return xbox.getRawAxis(XboxLeftXStick);
    }

    public static double getLeftXboxY() {
        return xbox.getRawAxis(XboxLeftYStick);
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

    public static double getRightXboxY() {
        return xbox.getRawAxis(XboxRightYStick);
    }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        turret.setDefaultCommand(new TurretSwitching(turret));
        // Configure the button bindings
        configureButtonBindings();
        if (Robot.debug) {
            startFireLog();
        }
    }
}

