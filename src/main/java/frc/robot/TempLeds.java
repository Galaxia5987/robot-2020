package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class TempLeds {
    private boolean povl_last = false;
    private Timer climb_leds_timer = new Timer();
    private Timer shift_leds_timer = new Timer();

    private AddressableLED m_led = new AddressableLED(0);     // Must be a PWM header, not MXP or DIO
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(22);

    private int m_rainbowFirstPixelHue;

    private final Turret turret;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    public TempLeds(Turret turret, Drivetrain drivetrain, Shooter shooter) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        m_led.setLength(m_ledBuffer.getLength());
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

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

    public void autonomousPeriodic() {
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180, 255, 128);
        }

        m_led.setData(m_ledBuffer);
    }

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
            if (shooter.isShooterReady() && shooter.getSpeed() > 5) {
                a = 100;
            } else if (RobotContainer.turret.isTurretReady() && VisionModule.leds.getBoolean(true)) {
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
                    if(drivetrain.isShiftedHigh())
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
                    if(drivetrain.isShiftedHigh())
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

    public void onShift() {
        shift_leds_timer.start();
    }

}
