/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.FullLocalization;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.CenterTurret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static final Joystick rightJoystick = new Joystick(0);
    public static final Joystick leftJoystick = new Joystick(1);
    public static final int rightYStick = 5;
    public static final double TURRET_JOYSTICK_SPEED = 1; //Coefficient of the joystick value per degree.
    public static final int XboxLeftXStick = 0;
    public static final int XboxLeftYStick = 1;
    public static final Climber climber = new Climber();
    private static final Intake intake = new Intake();
    private static final Turret turret = new Turret();
    private static final XboxController xbox = new XboxController(2);
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    // The robot's subsystems and commands are defined here...
    private final Drivetrain drivetrain = new Drivetrain();
    private final ColorWheel colorWheel = new ColorWheel();
    private final JoystickButton rightJoystickButton3 = new JoystickButton(rightJoystick, 3);
    private final Shooter shooter = new Shooter();
    private static Conveyor conveyor = new Conveyor();
    private final JoystickButton a = new JoystickButton(xbox, 3);
    private final JoystickButton b = new JoystickButton(xbox, 4);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
            new ValueTuner().start();
        }
    }

    public static double getLeftXboxY() {
        return xbox.getRawAxis(XboxLeftYStick);
    }

    public static double getRightXboxY() {
        return xbox.getRawAxis(rightYStick);
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

    public double getLeftXboxX() {
        return xbox.getRawAxis(XboxLeftXStick);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        rightJoystickButton3.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        a.whenPressed(new TurnTurret(turret, 45));
        b.whenPressed(new CenterTurret(turret));
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
