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
/*| Hours wasted on writing this code: 15                                |*/
/*+----------------------------------------------------------------------+*/


package frc.robot; //Package Declaration(I think thats what this...)

//Import Statements
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.CameraControl;
import frc.robot.Camera;

//Main Robot Class
public class Robot extends TimedRobot
{
  //PWM Connections for the mecanum drive
  private static final int PWM_FRONT_LEFT = 0; //Front Left (PWM 0)
  private static final int PWM_REAR_LEFT = 1; //Front Right (PWM 1)
  private static final int PWM_FRONT_RIGHT = 2; //Rear Right (PWM 2)
  private static final int PWM_REAR_RIGHT = 3; //Rear Left (PWM 3)

  //PWM Connections for the driver camera control
  private static final int PWM_DRIVER_CAMERA_X = 4; //Driver Camera X (PWM 4)
  private static final int PWM_DRIVER_CAMERA_Y = 5; //Driver Camera Y (PWM 5)

  private static final int PWM_ARM_CAMERA_X = 6; //Arm Camera X (PWM 6)
  private static final int PWM_ARM_CAMERA_Y = 7; //Arm Camera Y (PWM 7)
  
  //PWM Connections for the ramp
  private static final int PWM_RAMP_MOTOR = 8;

  //Create the ESC objects for the Mecanum Drive
  private Spark frontLeft = new Spark(PWM_FRONT_LEFT); //Front Left ESC
  private Spark rearLeft = new Spark(PWM_REAR_LEFT); //Rear Left ESC
  private Spark frontRight = new Spark(PWM_FRONT_RIGHT); //Front Right ESC
  private Spark rearRight = new Spark(PWM_REAR_RIGHT); //Rear Right ESC
  
  //Create the drive object
  private MecanumDrive robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  
  //Camera Control Objects
  private CameraControl driverCam = new CameraControl(PWM_DRIVER_CAMERA_X, PWM_DRIVER_CAMERA_Y);

  //Create the ESC object for the ramp
  private static Talon rampMotor = new Talon(PWM_RAMP_MOTOR);

  //Joysticks/Controllers
  private Joystick driveControl = new Joystick(0); //Movement and Driver Camera Control
  private XboxController armControl = new XboxController(1); //Arms and Arm Camera Control

  //Timer Objects
  private Timer timer = new Timer();

  //Power Distribution Board
  public static CAN pduCAN = new CAN(0);
  public static PowerDistributionPanel pdu = new PowerDistributionPanel(0);

  //Cameras
 
  //Pneumatics
  public static CAN pcmCAN = new CAN(1);
  public static Compressor c = new Compressor(0); //Air Compressor
  public static Solenoid s1 = new Solenoid(0);
  public static Solenoid s2 = new Solenoid(1);
  public static Solenoid s3 = new Solenoid(2);
  public static Solenoid s4 = new Solenoid(3);
  public static boolean solenoidOneExtended = false;
  public static boolean solenoidTwoExtended = false;

  //Pressure readings
  AnalogInput airPressure = new AnalogInput(0);
  
  public static int egg = 0;

  //Cameras
  
  

  @Override
  public void robotInit()
  {
    c.setClosedLoopControl(true); //Air compressor
    
    //Multithreading for live camera feed from multiple cameras
    //If you do not know how this works, DO NOT FUCK AROUND WITH
    //ANY PART OF THIS CODE!!
      // new Thread(() -> {
      //     UsbCamera driver = CameraServer.getInstance().startAutomaticCapture(1);
      //     driver.setResolution(640, 480);
      //     driver.setFPS(30);
      //     CvSink cvSinkDriver = CameraServer.getInstance().getVideo();
      //     CvSource outputStreamDriver = CameraServer.getInstance().putVideo("Driver", 640, 480);
      //     Mat sourceDriver = new Mat();
      //     Mat outputDriver = new Mat();
      //     while(!Thread.interrupted()) 
      //     {
      //       cvSinkDriver.grabFrame(sourceDriver);
      //       Imgproc.cvtColor(sourceDriver, outputDriver, Imgproc.COLOR_BGR2GRAY);
      //       outputStreamDriver.putFrame(outputDriver);
      //     }
      // }).start();

      new Thread(() -> {
        UsbCamera arms = CameraServer.getInstance().startAutomaticCapture(1);
        arms.setResolution(320, 240);
        arms.setFPS(30);
        CvSink cvSinkArms = CameraServer.getInstance().getVideo();
        CvSource outputStreamArms = CameraServer.getInstance().putVideo("Arms", 1280, 720);
        Mat sourceArms = new Mat();
        Mat outputArms = new Mat();
        while(!Thread.interrupted()) 
        {
          cvSinkArms.grabFrame(sourceArms);
          Imgproc.cvtColor(sourceArms, outputArms, Imgproc.COLOR_BGR2GRAY);
          outputStreamArms.putFrame(outputArms);
        }
      }).start();

  //     UsbCamera drivercam =  CameraServer.getInstance().startAutomaticCapture(0);
  //     UsbCamera armcam = CameraServer.getInstance().startAutomaticCapture(1);
  //     armcam.setFPS(30);
  //     armcam.setResolution(1280, 720);
  //     drivercam.setFPS(30);
  //     drivercam.setResolution(1280, 720);
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

    //GET DATA
    double channel12I = pdu.getCurrent(12); //Get current of front left ESC
    double channel14I = pdu.getCurrent(14); //Get current of rear left ESC
    double channel13I = pdu.getCurrent(13); //Get current of front right ESC
    double channel15I = pdu.getCurrent(15); //Get current of rear right ESC
    double x = driveControl.getX(); //Get joystick x
    double y = -driveControl.getY(); //Get joysitck y
    double z = driveControl.getZ(); //Get joystick z
    double aPv;
    double aP;
    double math;
    double maath;
    aPv = airPressure.getVoltage();
    math = aPv / 5;
    maath = 250 * math;
    aP = maath - 25;


    //Setup the Shuffleboard(SB)
    SmartDashboard.putData("RobotDrive: ", robotDrive); //Add Robot Drive to SB
    SmartDashboard.putNumber("Front Left Current: ", channel12I); //Add I draw of front left to SB
    SmartDashboard.putNumber("Left Rear Current: ", channel14I); //Add I draw of rear left to SB
    SmartDashboard.putNumber("Front Right Current: ", channel13I); //Add I draw of front right to SB
    SmartDashboard.putNumber("Rear Right Current: ", channel15I); //Add I draw of rear right to SB
    SmartDashboard.putNumber("Joystick X: ", x); //Add joystick X val to SB
    SmartDashboard.putNumber("Joystick Y: ", y); //Add joystick Y val to SB
    SmartDashboard.putNumber("Joystick Z: ", z);  //Add joystick Z val to SB
    SmartDashboard.putNumber("Air Pressure", aP); //Add air pressure to SB

    c.setClosedLoopControl(true); 
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
    double aPv;
    double aP;
    double math;
    double maath;
    aPv = airPressure.getVoltage();
    math = aPv / 5;
    maath = 250 * math;
    aP = maath - 25;


    //Setup the Shuffleboard(SB)
    SmartDashboard.putData("RobotDrive: ", robotDrive); //Add Robot Drive to SB
    SmartDashboard.putNumber("Front Left Current: ", channel12I); //Add I draw of front left to SB
    SmartDashboard.putNumber("Left Rear Current: ", channel14I); //Add I draw of rear left to SB
    SmartDashboard.putNumber("Front Right Current: ", channel13I); //Add I draw of front right to SB
    SmartDashboard.putNumber("Rear Right Current: ", channel15I); //Add I draw of rear right to SB
    SmartDashboard.putNumber("Joystick X: ", x); //Add joystick X val to SB
    SmartDashboard.putNumber("Joystick Y: ", y); //Add joystick Y val to SB
    SmartDashboard.putNumber("Joystick Z: ", z);  //Add joystick Z val to SB
    SmartDashboard.putNumber("Air Pressure", aP); //Add air pressure to SB

    //Move the robot
    robotDrive.driveCartesian(x, y, z);

    //Move the cameras
    driverCam.look(driveControl.getPOV());

    if (driveControl.getRawButton(5))
    {
      //Check if pistons are retracted, if yes, then extend them
      if (solenoidOneExtended == false)
      {
        s1.set(true);
        s2.set(false);
        s3.set(true);
        s4.set(false);
        solenoidOneExtended = true;
        solenoidTwoExtended = true;
      }

      //check if pistons are extended, if yes, then retract them
      if (solenoidOneExtended == true)
      {
        s1.set(false);
        s2.set(true);
        s3.set(false);
        s4.set(true);
        solenoidOneExtended = false;
        solenoidTwoExtended = false;
      }
    }

    // Control Ramp
    if (driveControl.getRawButton(3))
    {
      // Open ramp
      // TODO: Prevent further motion if stop switch is activated
      rampMotor.set(0.8);
    }

    else if (driveControl.getRawButton(4))
    {
      // Close ramp
      // TODO: Prevent futher motion if stop switch is activated
      rampMotor.set(-0.8);
    }
    else
    {
      // Stop moving ramp if no button is pressed
      rampMotor.set(0);
    }
  }
}