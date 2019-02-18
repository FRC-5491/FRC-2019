package frc.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Class for controlling cameras via SPI and launchpads
 */
public class CameraControl
{

  private int direction;
  private static SPI spiTransmission = new SPI(Port.kOnboardCS0);
  //SPI COMMAND CODES---------------------------------------------------
  private static byte[] stopCode = new byte[0x00]; //Stop
  private static byte[] armCamUp = new byte[0x01]; //Arm Cam Up
  private static byte[] armCamDown = new byte[0x02]; //Arm Cam Down
  private static byte[] armCamLeft = new byte[0x03]; //Arm Cam Left
  private static byte[] armCamRight = new byte[0x04]; //Arm Cam Right
  private static byte[] driverCamUp = new byte[0x05]; //Driver Cam Up
  private static byte[] driverCamDown = new byte[0x06]; //Driver Cam Down
  private static byte[] driverCamLeft = new byte[0x07]; //Driver Cam Left
  private static byte[] driverCamRight = new byte[0x08]; //Driver Cam Right


  public CameraControl()
  {
  }

  public void driverLook(int direction)
  {
    this.direction = direction;
    
    if (direction == 0)
    {
      spiTransmission.write(driverCamUp, 1);
    }

    if (direction == 45)
    {
      spiTransmission.write(driverCamRight, 1);
      spiTransmission.write(driverCamUp, 1);
    }

    if (direction == 90)
    {
      spiTransmission.write(driverCamRight, 1);
    }

    if (direction == 135)
    {
      spiTransmission.write(driverCamRight, 1);
      spiTransmission.write(driverCamDown, 1);
    }

    if (direction == 180)
    {
      spiTransmission.write(driverCamDown, 1);
    }

    if (direction == 225)
    {
      spiTransmission.write(driverCamLeft, 1);
      spiTransmission.write(driverCamDown, 1);
    }

    if (direction == 270)
    {
      spiTransmission.write(driverCamLeft, 1);
    }

    if (direction == 315)
    {
      spiTransmission.write(driverCamLeft, 1);
      spiTransmission.write(driverCamUp, 1);
    }
  }
}