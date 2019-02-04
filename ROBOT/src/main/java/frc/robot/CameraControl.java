package frc.robot;
import edu.wpi.first.wpilibj.*;

public class CameraControl
{
  int xAxis;
  int yAxis;
  double angleX;
  double angleY;
  Servo x;
  Servo y;
  int direction;

  public CameraControl(int xAxis, int yAxis)
  {
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    x = new Servo(xAxis);
    y = new Servo(yAxis);
  }

  public double getAngleX()
  {
    angleX = x.getAngle();
    return angleX;
  }

  public double getAngleY()
  {
    angleY = y.getAngle();
    return angleY;
  }


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