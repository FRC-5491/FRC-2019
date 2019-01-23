/*+----------------------------------------------------------------------+*/
/*| Explanation of Code:                                                 |*/
/*| Written for Team 5491's 2019 DESTINATION: DEEP SPACE FRC Competition |*/
/*| and used on their robot, Johnny.                                     |*/
/*+----------------------------------------------------------------------+*/
/*| Author(s): Jack Pirone, Kathryn Adinolfi, Julien Blanchet            |*/
/*+----------------------------------------------------------------------+*/
/*| Copyright: Jack Pirone, 2019. SEE BELOW                              |*/
/*| This is free sotware.                                                |*/
/*| It can be modified and distributed as long as credit is given to the |*/
/*| orignal authors, and if the distributer specifies that changes were  |*/
/*| to the original program.                                             |*/
/*+----------------------------------------------------------------------|*/
/*| Hours wasted on writing this code: 10                                |*/
/*+----------------------------------------------------------------------+*/


package frc.robot; //Package Declaration(I think thats what this...)

//Import Statements
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;

//Main Robot Class
public class Robot extends TimedRobot
{
  //PWM Connections for the mecanum drive
  private static final int PWM_FRONT_LEFT = 0; //Front Left (PWM 0)
  private static final int PWM_REAR_LEFT = 1; //Front Right (PWM 1)
  private static final int PWM_FRONT_RIGHT = 2; //Rear Right (PWM 2)
  private static final int PWM_REAR_RIGHT = 3; //Rear Left (PWM 3)

  //PWM Connections for the camera control
  private static final int PWM_CAMERA_X = 4; //Camera X (PWM 4)
  private static final int PWM_CAMERA_Y = 5; //Camera Y (PWM 5)

  //PWM Connections for the ramp
  private static final int PWM_RAMP_MOTOR = 6;

  //Create the ESC objects for the Mecanum Drive
  private Spark frontLeft = new Spark(PWM_FRONT_LEFT); //Front Left ESC
  private Spark rearLeft = new Spark(PWM_REAR_LEFT); //Rear Left ESC
  private Spark frontRight = new Spark(PWM_FRONT_RIGHT); //Front Right ESC
  private Spark rearRight = new Spark(PWM_REAR_RIGHT); //Rear Right ESC

  //Create the drive object
  private MecanumDrive robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  
  //Camera Control Objects
  private Servo cameraX = new Servo(PWM_CAMERA_X); //Up/Down Axis
  private Servo cameraY = new Servo(PWM_CAMERA_Y); //Left/Right Axis

  //Create the ESC object for the ramp
  private static Talon rampMotor = new Talon(PWM_RAMP_MOTOR);

  //Joysticks/Controllers
  private Joystick driveControl = new Joystick(0); //Joystick for Camera/Arm
  //private XboxController movementControl = new XboxController(0);

  //Timer Objects
  private Timer timer = new Timer();

  //Power Distribution Board
  public static CAN pduCAN = new CAN(0);
  public static PowerDistributionPanel pdu = new PowerDistributionPanel(0);

  public static int egg = 0;


  @Override
  public void robotInit()
  {
    cameraX.setAngle(90); //Set CameraX to 90 Degrees
    cameraY.setAngle(90); //Set CameraY to 90 Degrees
    
  }

  @Override
  public void autonomousInit()
  {
    timer.reset(); //Reset the timer
    timer.start(); //Start the timer
  }

  @Override
  public void autonomousPeriodic()
  {
    //DON'T PUT THINGS IN HERE
  }

  @Override
  public void teleopInit()
  {
  }
  
  @Override
  public void teleopPeriodic()
  { 
    //GET DATA
    double channel12I = pdu.getCurrent(12); //Get current of front left ESC
    double channel14I = pdu.getCurrent(14); //Get current of rear left ESC
    double channel13I = pdu.getCurrent(13); //Get current of front right ESC
    double channel15I = pdu.getCurrent(15); //Get current of rear right ESC
    double x = driveControl.getX(); //Get joystick x
    double y = -driveControl.getY(); //Get joysitck y
    double z = driveControl.getZ(); //Get joystick z

    //Setup the Shuffleboard(SB)
    SmartDashboard.putData("RobotDrive: ", robotDrive); //Add Robot Drive to SB
    SmartDashboard.putNumber("Front Left Current: ", channel12I); //Add I draw of front left to SB
    SmartDashboard.putNumber("Left Rear Current: ", channel14I); //Add I draw of rear left to SB
    SmartDashboard.putNumber("Front Right Current: ", channel13I); //Add I draw of front right to SB
    SmartDashboard.putNumber("Rear Right Current: ", channel15I); //Add I draw of rear right to SB
    SmartDashboard.putNumber("Joystick X: ", x); //Add joystick X val to SB
    SmartDashboard.putNumber("Joystick Y: ", y); //Add joystick Y val to SB
    SmartDashboard.putNumber("Joystick Z: ", z);  //Add joystick Z val to SB

    //Move the robot
    robotDrive.driveCartesian(x, y, z);

    //Camera Movement
    if (driveControl.getPOV() == 0) //Camera Up
    {
      cameraY.setAngle(cameraY.getAngle() + 1); 
    }

    if (driveControl.getPOV() == 45) //Camera Up+Right
    {
      cameraX.setAngle(cameraX.getAngle() + 1); 
      cameraY.setAngle(cameraY.getAngle() + 1);
    }

    if (driveControl.getPOV() == 90) //Camera Right
    {
      cameraX.setAngle(cameraX.getAngle() + 1);
    }

    if (driveControl.getPOV() == 135) //Camera Down+Right
    {
      cameraX.setAngle(cameraX.getAngle() + 1);
      cameraY.setAngle(cameraY.getAngle() - 1);
    }

    if (driveControl.getPOV() == 180) //Camera Down
    {
      cameraY.setAngle(cameraY.getAngle() - 1);
    }

    if (driveControl.getPOV() == 225) //Camera Down+Left
    {
      cameraX.setAngle(cameraX.getAngle() - 1);
      cameraY.setAngle(cameraY.getAngle() - 1);
    }

    if (driveControl.getPOV() == 270) //Camera Left
    {
      cameraX.setAngle(cameraX.getAngle() - 1);
    }

    if (driveControl.getPOV() == 315) //Camera Up+Left
    {
      cameraX.setAngle(cameraX.getAngle() - 1);
      cameraY.setAngle(cameraY.getAngle() + 1);
    }
  }
}