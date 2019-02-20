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
/*| Hours wasted on writing this code: 17                                |*/
/*+----------------------------------------------------------------------+*/


package frc.robot; //Package Declaration(I think thats what this...)

//Import Statements

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.CameraControl;
import frc.robot.ArmControl;

//Main Robot Class
public class Robot extends TimedRobot
{
  //VARIABLE DECLARATION ---------------------------------------------------

  //JOYSTICK DEADBAND
  private static final double DEADBAND = 0.05;

  //PWM Connections for Mecanum Drive
  private static final int PWM_FRONT_LEFT = 0; //Front Left (PWM 0)
  private static final int PWM_REAR_LEFT = 1; //Front Right (PWM 1)
  private static final int PWM_FRONT_RIGHT = 2; //Rear Right (PWM 2)
  private static final int PWM_REAR_RIGHT = 3; //Rear Left (PWM 3)

  //PWM RAMP MOTOR
  private static final int PWM_RAMP_MOTOR = 4;

  //PWM Connections for Arm Control
  private static final int PWM_ARM_HEIGHT = 7; //Arm Height Up/Down
  private static final int PWM_ARM_TILT = 8; //Arm Tilt Up/Down
  private static final int PWM_ARM_BALL = 9; //Fetch/Eject Ball


  //PWM Connections for cameras -- These are on the MXP PORT
  private static final int DRIVE_CAM_X = 10; //Drive cam servo x -- MXP 11
  private static final int DRIVE_CAM_Y = 11; //Driver cam servo y -- MXP 13
  private static final int ARM_CAM_X = 12; //Arm cam servo x -- MXP 15
  private static final int ARM_CAM_Y = 13; //Arm cam servo y -- MXP 17

  

  // PCM Connections for Solenoids
  private static final int PCM_ARM_EJECT_ONE = 0;
  private static final int PCM_ARM_EJECT_TWO = 1;

  // Digital IO Connections
  private static final int DIG_IO_TILT_SWITCH = 0;
  
  

  //OBJECT CREATION -----------------------------------------------------

  //Create the ESC objects for the Mecanum Drive
  private Spark frontLeft = new Spark(PWM_FRONT_LEFT); //Front Left ESC
  private Spark rearLeft = new Spark(PWM_REAR_LEFT); //Rear Left ESC
  private Spark frontRight = new Spark(PWM_FRONT_RIGHT); //Front Right ESC
  private Spark rearRight = new Spark(PWM_REAR_RIGHT); //Rear Right ESC
  
  //Create the drive object
  private MecanumDrive robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight); //Drive Control Object

  //Create the ramp ESC
  private Talon rampMotor = new Talon(PWM_RAMP_MOTOR); //Ramp ESC

  //Joysticks/Controllers
  private Joystick driveControl = new Joystick(0); //Movement and Driver Camera Control
  private XboxController armControl = new XboxController(1); //Arms and Arm Camera Control

  //Cameras
  private CameraControl driverCam = new CameraControl(DRIVE_CAM_X, DRIVE_CAM_Y); //Driver cam control
  private CameraControl armCam = new CameraControl(ARM_CAM_X, ARM_CAM_Y); //Arm cam control
 
  //Timer Objects
  private Timer timer = new Timer();

  //Power Distribution Board
  public static PowerDistributionPanel pdu = new PowerDistributionPanel(0); //PDU on CANBUS 0

  //Pneumatics
  public static Compressor c = new Compressor(0); //Air Compressor
  AnalogInput airPressure = new AnalogInput(0); //Pressure readings

  DigitalInput switch1 = new DigitalInput(0); // Limit Switch 1
  DigitalInput switch2 = new DigitalInput(1); // Limit Switch 2
  
  public static int egg = 0; //EDUCATION

  //Arm Control
  public static ArmControl arms = new ArmControl(PWM_ARM_HEIGHT, PWM_ARM_TILT, PWM_ARM_BALL, PCM_ARM_EJECT_ONE, PCM_ARM_EJECT_TWO); //Arm Control Object

  //BEGIN ROBOT CODE --------------------------------------------------------------

  @Override
  public void robotInit()
  {
    robotDrive.setDeadband(DEADBAND);
    c.setClosedLoopControl(true); //Air compressor

    //GET DATA
    double channel12I = pdu.getCurrent(12); //Get current of front left ESC
    double channel14I = pdu.getCurrent(14); //Get current of rear left ESC
    double channel13I = pdu.getCurrent(13); //Get current of front right ESC
    double channel15I = pdu.getCurrent(15); //Get current of rear right ESC
    double x = driveControl.getX(); //Get joystick x
    double y = -driveControl.getY(); //Get joysitck y
    double z = driveControl.getZ(); //Get joystick z
    
    //Calculate Air Pressure
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
  {int e;
    
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
    
    //Calculate Air Pressure
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
    
    SmartDashboard.putBoolean("Switch 1", switch1.get());
    SmartDashboard.putBoolean("Switch 2", switch2.get());
    
    double driveX = driveControl.getX(); //Get joystick x
    double driveY = -driveControl.getY(); //Get joysitck y
    double driveZ = driveControl.getZ(); //Get joystick z

    double armLeftX = armControl.getX(Hand.kLeft); //Get xbox LS x
    double armLeftY = -armControl.getY(Hand.kLeft); //Get xbox LS y
    
    double armRightX = armControl.getX(Hand.kRight); //Get xbox RS x
    double armRightY = -armControl.getY(Hand.kRight); //Get xbox RS y

    double armTriggerL = armControl.getTriggerAxis(Hand.kLeft); //Get xbox LT val
    double armtriggerR = armControl.getTriggerAxis(Hand.kRight); //Get xbox RT val

    //Move the robot
    robotDrive.driveCartesian(driveX, driveY, driveZ);

    //Move the arms
    if(armLeftY >= 0.15) {
      arms.moveArms(armLeftY);
    } else if (armLeftY <= -0.15) {
      arms.moveArms(armLeftY);
    } else {
      arms.moveArms(0.0);
    }

    //Tilt the arms
    if (armTriggerL > 0.25) {
      arms.tiltArms(0.5);
    } else if (armtriggerR > 0.25 && switch1.get()) {
      arms.tiltArms(-0.5);
    } else {
      arms.tiltArms(0.0);
    }

    //Eject ball
    if (armControl.getBumper(Hand.kRight)) {
      arms.ejectBall();
    } else if (armControl.getBumper(Hand.kLeft)) {
      arms.fetchBall();
    } else {
      arms.stopBallMotors();
    }

     //Pancakes
     if (armControl.getBButton()) {
      arms.ejectPancakeExtend();
     } else if (armControl.getYButton()){
       arms.ejectPancakeRetract();
     } else {
       arms.ejectPancakeStop();
     }

    //Flip Ramp
    if(driveControl.getRawButton(10)) {
      rampMotor.set(1.0);
    } else if (driveControl.getRawButton(11)) {
      rampMotor.set(-1.0);
    } else {
      rampMotor.set(0.0);
    }
    //END OF TELEOP PERIODIC
  }

  @Override public void disabledInit()
  {    
    
  }
}