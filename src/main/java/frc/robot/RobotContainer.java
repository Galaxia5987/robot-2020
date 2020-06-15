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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commnads.DimmingColor;
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

import static frc.robot.Constants.LED.DEFAULT_COLOR;
import static frc.robot.Ports.LED.STRIP_LENGTH;

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
    public static final LED led = new LED(STRIP_LENGTH);
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
        colorWheel.setDefaultCommand(new ManualControl(colorWheel));
        turret.setDefaultCommand(new JoystickTurret(turret));
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain));
        led.setDefaultCommand(new DimmingColor(led, DEFAULT_COLOR, 5, 99999,false));
    }
    /**
     * Configures all of the button usages on the robot.
     */
    private void configureButtonBindings() {
        OI.a.whileHeld(new FeedTurret(conveyor, shooter::isShooterReady, turret::isTurretReady, shooter::isShooting));
        OI.x.whileHeld(new OuttakeBalls(conveyor, intake));
        OI.b.toggleWhenPressed(new SpeedUp(shooter));
        OI.y.whileHeld(new PickupBalls(intake, conveyor));
        OI.back.whenPressed(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        OI.rb.whenPressed(new RotationControl(colorWheel));
        OI.lb.whenPressed(new PositionControl(colorWheel));
        OI.back_start.whenHeld(new SequentialCommandGroup(
                new WaitCommand(2),
                new RunCommand(() -> Robot.shootingManualMode = true)
        )); //If both buttons are held without being released the manualMode will be enabled.
        OI.start.whenPressed(() -> Robot.shootingManualMode = false); //Pressing start disables the manual mode for shooting.
        for (int i = 1; i < 10; i++) {
            new JoystickButton(OI.leftStick, i).whenPressed(new GearShift(drivetrain, Drivetrain.shiftModes.HIGH));
        }
        for (int i = 1; i < 10; i++) {
            new JoystickButton(OI.rightStick, i).whenPressed(new GearShift(drivetrain, Drivetrain.shiftModes.LOW));
        }
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