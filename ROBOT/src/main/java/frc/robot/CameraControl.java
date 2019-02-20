package frc.robot;
import edu.wpi.first.wpilibj.*;

/**
 * Class for controlling cameras via PWM
 * 
 */
public class CameraControl
{

  private int direction;
  private int PWMX;
  private int PWMY;
  private Servo x;
  private Servo y;
  

  /**
   * Constructor for CameraControl object.
   * @param PWMX (Int) PWM channel for camera X servo
   * @param PWMY (Int) PWM channel for camera y servo
   */
  public CameraControl(int PWMX, int PWMY)
  {
    this.PWMX = PWMX;
    this.PWMY = PWMY;
    x = new Servo(PWMX);
    y = new Servo(PWMY);
  }

  /**
   * Look around with camera
   * @param direction (Int) -- Takes a values 0, 45, 90, 135, 180, 225, 270, 315
   */
  public void look(int direction)
  {
    this.direction = direction;
    
    if (direction == 0)
    {
      y.setAngle(y.getAngle() + 1);
    }

    if (direction == 45)
    {
      x.setAngle(x.getAngle() + 1);
      y.setAngle(y.getAngle() + 1);
    }

    if (direction == 90)
    {
      x.setAngle(x.getAngle() + 1);
    }

    if (direction == 135)
    {
      x.setAngle(x.getAngle() + 1);
      y.setAngle(y.getAngle() - 1);
      
    }

    if (direction == 180)
    {
      y.setAngle(y.getAngle() - 1);
    }

    if (direction == 225)
    {
      x.setAngle(x.getAngle() - 1);
      y.setAngle(y.getAngle() - 1);
    }

    if (direction == 270)
    {
      x.setAngle(x.getAngle() - 1);
    }

    if (direction == 315)
    {
      x.setAngle(x.getAngle() - 1);
      y.setAngle(y.getAngle() + 1);
    }
  }
}