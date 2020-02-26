/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.TrajectoryLoader;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

import static frc.robot.RobotContainer.turret;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final boolean debug = isDebug() && !DriverStation.getInstance().isFMSAttached();
    // The roboRIO has built-in pull up resistors, bridge signal and ground pins on Robot A DIO 0.
    public static boolean isRobotA = !new DigitalInput(0).get();
    public static boolean shootingManualMode = false;
    public static Compressor compressor = new Compressor();
    public static PowerDistributionPanel pdp = new PowerDistributionPanel();
    public static Timer robotTimer = new Timer();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private boolean povl_last = false;
    private Timer climb_leds_timer = new Timer();
    public static Timer shift_leds_timer = new Timer();

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;

    /**
     * @return Robot in debug mode
     */
    private static boolean isDebug() {
        return false;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (!Robot.isRobotA) {
            Utils.swapConstants(Constants.class, BConstants.class);
            Utils.swapConstants(Ports.class, BPorts.class);
        }
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotTimer.reset();
        robotTimer.start();

        TrajectoryLoader.loadTrajectories();

        m_robotContainer = new RobotContainer();
        CustomDashboard.setAutonomousModes(m_robotContainer.getAutonomousModes());

//        compressor.stop();

        SmartDashboard.putBoolean("Robot A", isRobotA);
        SmartDashboard.putBoolean("Debug", debug);

//        PortForwarder.add(1181, "10.59.87.7", 1181);
        PortForwarder.add(8888, "10.59.87.7", 5800);
        PortForwarder.add(8887, "10.59.87.7", 22);

        //Disable live window for more loop time
        LiveWindow.setEnabled(false);
        LiveWindow.disableAllTelemetry();

        startCameraCapture();
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(22);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void startCameraCapture() {
        new Thread(() -> {
            UsbCamera camera = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(0);
            camera.setResolution(160, 120);
            camera.setFPS(30);
        }).start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();


    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        VisionModule.setLEDs(false);
    }

    @Override
    public void disabledPeriodic() {

        m_rainbowFirstPixelHue += 10;
        m_rainbowFirstPixelHue %= 360;
        int hue;
        switch(DriverStation.getInstance().getAlliance()){
            case Red:
                hue = 0;
                break;
            case Blue:
                hue = 122;
                break;
            default:
                hue = 95;

        }
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hue, 255, 30 + (int)(60*(Math.sin(Math.toRadians(m_rainbowFirstPixelHue))+1)/2));
        }

        m_led.setData(m_ledBuffer);

    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        turret.setPower(0);
        m_robotContainer.drivetrain.setBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180, 255, 128);
        }

        m_led.setData(m_ledBuffer);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.drivetrain.setBrake(false);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        //Toggle on the left arrow to start and reset the timer
        if(OI.povl.get() && !povl_last){
            if(climb_leds_timer.get() == 0){ //hasn't started already
                climb_leds_timer.reset();
                climb_leds_timer.start();
            }
            else{//already on
                climb_leds_timer.stop();
                climb_leds_timer.reset();
            }
        }

        //This is a value which allows the colors to race around the robot.
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 30;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int hue = (m_rainbowFirstPixelHue + (i * 60 / m_ledBuffer.getLength())) % 30;
            int a;

            //The order goes like this: Are we climbing? Is the shooter ready? Is the turret ready? are the LED's on?
            if (m_robotContainer.shooter.isShooterReady() && m_robotContainer.shooter.getSpeed() > 5) {
                a = 100;
            } else if (m_robotContainer.turret.isTurretReady() && VisionModule.leds.getBoolean(true)) {
                a = 20;
            } else if (VisionModule.leds.getBoolean(true)){
                a = 50;
            }
            else{
                a = -1;
            }

            if(climb_leds_timer.get() != 0){ //climbing code
                if(climb_leds_timer.get()>=3)
                    m_ledBuffer.setHSV(i, 60*m_rainbowFirstPixelHue, 255, 255); //TODO: the hue is rainbows, find something cool or leave it
                else {
                    int climb_hue = (int) (100 - 45 * Math.floor(climb_leds_timer.get()));
                    m_ledBuffer.setHSV(i, climb_hue, 255, (int) (55 + 200 * (1 - climb_leds_timer.get() % 1))); //fade the color along with the timer
                }
            }
            else if(a != -1) {
                if(shift_leds_timer.get() != 0 && i%6 < 2) //6 and 2 are the amount of stripes and their width when shifting
                {
                    if(m_robotContainer.drivetrain.isShiftedHigh())
                        m_ledBuffer.setHSV(i, 100, 255, 80); //the high color
                    else
                        m_ledBuffer.setHSV(i, 0, 255, 80); //the high color
                }
                else
                m_ledBuffer.setHSV(i, hue + a, 255, 128);
            }
            else {
                if(shift_leds_timer.get() != 0 && i%6 < 2) //6 and 2 are the amount of stripes and their width when shifting
                {
                    if(m_robotContainer.drivetrain.isShiftedHigh())
                        m_ledBuffer.setHSV(i, 100, 255, 80); //the high color
                    else
                        m_ledBuffer.setHSV(i, 0, 255, 80); //the high color
                }
                else
                m_ledBuffer.setHSV(i, 0, 0, 0); //TODO: have something idle run when nothing is happening if youd like.
            }
        }
        povl_last = OI.povl.get();
        m_led.setData(m_ledBuffer);

        if(shift_leds_timer.get() >= 1){
            shift_leds_timer.stop();
            shift_leds_timer.reset();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
