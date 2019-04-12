package frc.robot;

import edu.wpi.first.wpilibj.Talon;
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

    //private Solenoid s1;
    //private Solenoid s2;

    private DigitalInput armTiltSwitch;

    /**
     * Constructor for the arm control class. Creates two talons
     * on PWM channels "pwmHeight" and "pwmTilt". Creates two
     * PWM controlled VictorSPX speed controller on pwm channels
     * "pwmBallLeft" and "pwmBallRight". Creates two solenoid objects on
     * PCM channels "pcmChannelOne" and "pcmChannelTwo".
     * 
     * @param pwmHeight (Int) PWM channel for talon controlling height.
     * @param pwmTilt (Int) PWM channel for talon controlling tilt.
     * @param pwmBall (Int) PWM channel for VictorSPX controlling ball motors.
     * @param pcmChannelOne (Int) PCM channel for solenoid extendtion. Defaults to PCM {@value (0)}.
     * @param pcmChannelTwo (Int) PCM channel for solenoid retraction Defaults to PCM {@value (1)}.
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
        //this.s1 = new Solenoid(pcmChannelOne);
        //this.s2 = new Solenoid(pcmChannelTwo);
 
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
    public void fetchBall()
    {
        ball.set(-0.5);
    }

    /**
     * Eject Ball
     */
    public void ejectBall()
    {
        ball.set(1.0);
    }

    /**
     * Eject Ball
     */
    public void stopBallMotors()
    {
        ball.set(0.0);
    }

    // public void ejectPancakeExtend()
    // {
    //     s1.set(true);
    //     s2.set(false);
    // }

    // public void ejectPancakeRetract()
    // {
    //     s1.set(false);
    //     s2.set(true);
    // }

    // public void ejectPancakeStop()
    // {
    //     s1.set(false);
    //     s2.set(false);
    // }

}