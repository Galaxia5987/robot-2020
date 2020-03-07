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
import frc.robot.autonomous.ResetOnly;
import frc.robot.autonomous.ShootAndDriveForward;
import frc.robot.autonomous.ShootAndDriveToPickup;
import frc.robot.autonomous.TrenchPickup;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.commandgroups.ProportionalPickup;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.commands.PIDClimbAndBalance;
import frc.robot.subsystems.climb.commands.ReleaseRods;
import frc.robot.subsystems.climb.commands.ResetClimber;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.ManualControl;
import frc.robot.subsystems.color_wheel.commands.PositionControl;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.GearShift;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.drivetrain.commands.ResetLocalization;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commandgroups.OuttakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.subsystems.turret.commands.TurretSwitching;
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
    public final VisionModule visionModule = new VisionModule();
    public final CustomDashboard customDashboard = new CustomDashboard();
    public final Drivetrain drivetrain = new Drivetrain();
    public final ColorWheel colorWheel = new ColorWheel();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final Conveyor conveyor = new Conveyor(intake);
    public static final Climber climber = new Climber();
    public static final Turret turret = new Turret();
    private final Command m_autoCommand = null;

    public TempLeds leds = new TempLeds(turret, drivetrain, shooter);

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
    }
    /**
     * Configures all of the button usages on the robot.
     */
    private void configureButtonBindings() {
        OI.a.whenPressed(new InstantCommand(() -> shooter.setHoodAngle(115)));
        OI.b.whenPressed(new InstantCommand(() -> shooter.setHoodAngle(0)));
        OI.x.whenPressed(new InstantCommand(() -> shooter.setHoodAngle(110)));
        OI.y.whenPressed(new InstantCommand(() -> shooter.setHoodAngle(120)));
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

    public String[] getAutonomousModes() {
        return new String[]{"trenchPickup", "shootAndDriveToPickup", "shootAndDriveForward", "resetOnly"};
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        String autoMode = CustomDashboard.getSelectedMode();
        switch (autoMode) {
            case "trenchPickup":
                return new TrenchPickup(shooter, conveyor, turret, drivetrain, intake);
            case "shootAndDriveToPickup":
                return new ShootAndDriveToPickup(turret, shooter, drivetrain, conveyor);
            case "shootAndDriveForward":
                return new ShootAndDriveForward(turret, shooter, drivetrain, conveyor);
            case "resetOnly":
                return new ResetOnly(drivetrain, turret);
        }
        return null;
    }

}