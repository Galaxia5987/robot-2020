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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.valuetuner.ValueTuner;
import org.techfire225.webapp.Webserver;
import frc.robot.subsystems.drivetrain.Drivetrain;

import frc.robot.subsystems.conveyor.Conveyor;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.CenterTurret;
import frc.robot.subsystems.turret.commands.JoystickTurret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.subsystems.shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    // The robot's subsystems and commands are defined here...
    private final Drivetrain drivetrain = new Drivetrain();
    private final ColorWheel colorWheel = new ColorWheel();
    private static Conveyor conveyor = new Conveyor();
    private static final Intake intake = new Intake();
    private static final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final XboxController xbox = new XboxController(2);
    private final JoystickButton a = new JoystickButton(xbox, 3);
    private final JoystickButton b = new JoystickButton(xbox, 4);
    public static final int rightYStick = 5;
    public static final double TURRET_JOYSTICK_SPEED = 1; //Coefficient of the joystick value per degree.


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

  /**
   * Initiates the value tuner.
   */
  private void startValueTuner() {
    new ValueTuner().start();
  }

  /**
   * Initiates the port of team 225s Fire-Logger.
   */
  private void startFireLog(){
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
  private void configureButtonBindings() {
      a.whenPressed(new RunCommand(() -> colorWheel.turnManual(false)));
  }

  public double getXboxY(){
    return xbox.getRawAxis(rightYStick);
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
