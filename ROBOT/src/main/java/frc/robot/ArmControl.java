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
    private int pwmBallLeft;
    private int pwmBallRight;
    private int pcmChannelOne;
    private int pcmChannelTwo;
    private long delayTime = 10;

    private int digitalIoTiltSwitch;

    private double armHeightSpeed;
    private double armTiltSpeed;

    private Talon height;
    private Talon tilt;
    private PWMVictorSPX ballLeft;
    private PWMVictorSPX ballRight;

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
     * @param pwmBallRight PWM channel for VictorSPX controlling right ball motor.
     * @param pwmBallLeft PWM channel for VictorSPX controlling left ball motor.
     * @param pcmChannelOne PCM channel for solenoid extendtion. Defaults to PCM {@value (0)}.
     * @param pcmChannelTwo PCM channel for solenoid retraction Defaults to PCM {@value (1)}.
     * @param digitalIoTiltSwitch Digital IO port used to check if arms are in ball mode or pancakce mode.
     * 
     */
    public ArmControl(int pwmHeight, int pwmTilt, int pwmBallLeft, int pwmBallRight, int pcmChannelOne, int pcmChannelTwo, int digitalIoTiltSwitch)
    {
        this.pwmHeight = pwmHeight;
        this.pwmTilt = pwmTilt;
        this.pwmBallLeft = pwmBallLeft;
        this.pwmBallRight = pwmBallRight;
        this.pcmChannelOne = pcmChannelOne;
        this.pcmChannelTwo = pcmChannelTwo;
        this.ballLeft = new PWMVictorSPX(pwmBallLeft);
        this.ballRight = new PWMVictorSPX(pwmBallLeft);
        this.tilt = new Talon(pwmTilt);
        this.height = new Talon(pwmHeight);
        this.s1 = new Solenoid(pcmChannelOne);
        this.s2 = new Solenoid(pcmChannelTwo);
        this.digitalIoTiltSwitch = digitalIoTiltSwitch;
        this.armTiltSwitch = new DigitalInput(digitalIoTiltSwitch);
 
    }

    /**
     * Move Arms Up.
     * @param armHeightSpeed (Double) Speed of arm movement. 
     */
    public void moveArmsUp(double armHeightSpeed)
    {
        height.set(armHeightSpeed);
    }

    /**
     * Move Arms Down.
     * @param armHeightSpeed (Double) Speed of arm movement. Must be a negative value 
     */
    public void moveArmsDown(double armHeightSpeed)
    {
        height.set(armHeightSpeed);
    }

    /**
     * Tilt Arms Up
     * @param armTiltSpeed (Double) Speed of arm movement.
     */
    public void tiltArmsUp(double armTiltSpeed)
    {
        tilt.set(armTiltSpeed);
    }

    /**
     * Tilt Arms Down
     * @param armTiltSpeed (Double) Speed of arm movement. Must be a negative value
     */
    public void tiltArmsDown(double armTiltSpeed)
    {
        tilt.set(armTiltSpeed);
    }

    /** 
     * Fetch Ball
     */
    public void fetchBall()
    {
        ballLeft.set(0.8);
        ballRight.set(0.8);
    }

    /**
     * Eject Ball
     */
    public void ejectBall()
    {
        ballLeft.set(-1.0);
        ballRight.set(1.0);
    }

    public void togglePancakeMode()
    {
        while(armTiltSwitch.get() == true)
        {
            tilt.set(0.5);
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