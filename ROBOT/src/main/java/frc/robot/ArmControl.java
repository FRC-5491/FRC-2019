package frc.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Class for controlling arms height, tilt, and ball collection/ejection, as well as getting the
 * height in inches, the tilt in degrees, and wether or not a ball is in the arms.
 */
public class ArmControl 
{
    private int pwmHeight;
    private int pwmTilt;
    private int pwmBall;
    private int pcmChannelOne;
    private int pcmChannelTwo;
    private long delayTime = 10;

    private Talon height;
    private Talon tilt;
    private PWMVictorSPX ball;

    private Solenoid s1;
    private Solenoid s2;

    private DigitalInput armTiltSwitch;

    /**
     * Constructor for the arm control class. Creates two talons
     * on PWM channels "pwmHeight" and "pwmTilt". Creates two
     * PWM controlled VictorSPX speed controller on pwm channels
     * "pwmBallLeft" and "pwmBallRight". Creates two solenoid objects on
     * PCM channels "pcmChannelOne" and "pcmChannelTwo".
     * 
     * @param pwmHeight PWM channel for talon controlling height.
     * @param pwmTilt PWM channel for talon controlling tilt.
     * @param pwmBall PWM channel for VictorSPX controlling ball motors.
     * @param pcmChannelOne PCM channel for solenoid extendtion. Defaults to PCM {@value (0)}.
     * @param pcmChannelTwo PCM channel for solenoid retraction Defaults to PCM {@value (1)}.
     * 
     */
    public ArmControl(int pwmHeight, int pwmTilt, int pwmBall, int pcmChannelOne, int pcmChannelTwo)
    {
        this.pwmHeight = pwmHeight;
        this.pwmTilt = pwmTilt;
        this.pwmBall = pwmBall;
        this.pcmChannelOne = pcmChannelOne;
        this.pcmChannelTwo = pcmChannelTwo;
        this.ball = new PWMVictorSPX(pwmBall);
        this.tilt = new Talon(pwmTilt);
        this.height = new Talon(pwmHeight);
        this.s1 = new Solenoid(pcmChannelOne);
        this.s2 = new Solenoid(pcmChannelTwo);
 
    }

    /**
     * Move Arms
     * @param armHeightSpeed (Double) Speed of arm movement. 
     */
    public void moveArms(double armHeightSpeed)
    {
        height.set(armHeightSpeed);
    }

    /**
     * Tilt Arms
     * @param armTiltSpeed (Double) Speed of arm movement.
     */
    public void tiltArms(double armTiltSpeed)
    {
        tilt.set(armTiltSpeed);
    }

    /** 
     * Fetch Ball
     */
    public void fetchBall() throws InterruptedException
    {
        ball.set(-0.8);
        wait(10, 1000);
        ball.set(0.0);
    }

    /**
     * Eject Ball
     */
    public void ejectBall() throws InterruptedException
    {
        try
        {
            ball.set(1.0);
            wait(10, 1000);
            ball.set(0.0);
        }catch(Exception e)
        {
            System.out.println("Delay Issues");
        }
    }

    public void ejectPancake() throws InterruptedException
    {
        try
        {
            s1.set(true);
            wait(10, 500);
            s1.set(false);
            s2.set(true);
            wait(10, 500);
            s2.set(false);
        }catch(Exception e)
        {
            System.out.println("Delay Issues");
        }
    }

}