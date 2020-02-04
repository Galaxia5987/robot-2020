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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.commands.CalculatedClimbAndBalance;
import frc.robot.subsystems.climb.commands.JoystickControl;
import frc.robot.subsystems.climb.commands.ReleaseRods;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.subsystems.turret.commands.TurretSwitching;
import frc.robot.utilities.StickButton;
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
    private final Drivetrain drivetrain = new Drivetrain();
    private final ColorWheel colorWheel = new ColorWheel();
    private final JoystickButton rightJoystickButton3 = new JoystickButton(rightJoystick, 3);
    private final Shooter shooter = new Shooter();
    public static JoystickButton a = new JoystickButton(xbox, 1);
    public static JoystickButton b = new JoystickButton(xbox, 2);
    public static JoystickButton y = new JoystickButton(xbox, 3);
    private final JoystickButton rs = new JoystickButton(xbox, 10);
    private final StickButton rightY = new StickButton(xbox, 5, 0.1);
    private static Conveyor conveyor = new Conveyor();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private void configureButtonBindings() {
        a.whileHeld(new JoystickControl(climber, false));
        b.whenPressed(new ReleaseRods(climber, 1.5));
        y.whenPressed(new CalculatedClimbAndBalance(climber, 1));
        rightY.whileHeld(new JoystickTurret(turret));
    }

    public static double getLeftXboxX() {
        return xbox.getRawAxis(XboxLeftXStick);
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

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public RobotContainer() {
        turret.setDefaultCommand(new TurretSwitching(turret, drivetrain));
        // Configure the button bindings
        configureButtonBindings();
        if (Robot.debug) {
            startValueTuner();
            startFireLog();
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
