/*+----------------------------------------------------------------------+*/
/*| Explanation of Code:                                                 |*/
/*| Written for Team 5491's 2019 DESTINATION: DEEP SPACE FRC Competition |*/
/*| and used on their robot, Johnny.                                     |*/
/*+----------------------------------------------------------------------+*/
/*| Author(s): Jack Pirone,                                              |*/
/*| Kathryn Adinolfi, Julien Blanchet                                    |*/
/*+----------------------------------------------------------------------+*/
/*| That human who fixed the autonomus method: Kelly Ostrom (Nutrons 125)|*/
/*+----------------------------------------------------------------------+*/
/*| Zip Tie Supplier: Henry/Hank Aumiller(Nutrons 125)                   |*/
/*+----------------------------------------------------------------------+*/
/*| Copyright: Jack Pirone, 2019. SEE BELOW                              |*/
/*| This is free sotware.                                                |*/
/*| It can be modified and distributed as long as credit is given to the |*/
/*| orignal authors, and if the distributer specifies that changes were  |*/
/*| to the original program.                                             |*/
/*+----------------------------------------------------------------------|*/
/*| Hours wasted on writing this code: 178                                |*/
/*+----------------------------------------------------------------------+*/


package frc.robot; //Package Declaration(I think thats what this...)

//Import Statements

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.CameraControl;
import frc.robot.ArmControl;

//Main Robot Class
public class Robot extends TimedRobot
{
  //VARIABLE DECLARATION ---------------------------------------------------

  //JOYSTICK DEADBAND
  private static final double DEADBAND = 0.1;

  //PWM Connections for Mecanum Drive
  private static final int PWM_FRONT_LEFT = 0; //Front Left (PWM 0)
  private static final int PWM_REAR_LEFT = 1; //Front Right (PWM 1)
  private static final int PWM_FRONT_RIGHT = 2; //Rear Right (PWM 2)
  private static final int PWM_REAR_RIGHT = 3; //Rear Left (PWM 3)

  //PWM RAMP MOTOR
  private static final int PWM_RAMP_MOTOR = 4; //Ramp motor (PWM 4)

  //PWM Connections for Arm Control
  private static final int PWM_ARM_HEIGHT = 7; //Arm Height Up/Down 
  private static final int PWM_ARM_TILT = 8; //Arm Tilt Up/Down
  private static final int PWM_ARM_BALL = 9; //Fetch/Eject Ball


  //PWM Connections for cameras -- These are on the MXP PORT
  private static final int DRIVE_CAM_X = 5; //Drive cam servo x PWM 5
  private static final int DRIVE_CAM_Y = 6; //Driver cam servo y PWM 6

  // PCM Connections for Solenoids
  private static final int PCM_ARM_EJECT_ONE = 1;
  private static final int PCM_ARM_EJECT_TWO = 0;

  // Digital IO Connections
  private static final int DIG_IO_TILT_SWITCH = 0;
  

  //OBJECT CREATION -----------------------------------------------------

  //Create the ESC objects for the Mecanum Drive
  private Spark frontLeft = new Spark(PWM_FRONT_LEFT); //Front Left ESC
  private Spark rearLeft = new Spark(PWM_REAR_LEFT); //Rear Left ESC
  private Spark frontRight = new Spark(PWM_FRONT_RIGHT); //Front Right ESC
  private Spark rearRight = new Spark(PWM_REAR_RIGHT); //Rear Right ESC
  
  // private SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft);
  // private SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);
  //Create the drive object
  private MecanumDrive robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  //Create the ramp ESC
  private Talon rampMotor = new Talon(PWM_RAMP_MOTOR); //Ramp ESC

  //Joysticks/Controllers
  private Joystick driveControl = new Joystick(0); //Movement and Driver Camera Control
  private XboxController armControl = new XboxController(1); //Arms and Arm Camera Control

  //Cameras
  private CameraControl driverCam = new CameraControl(DRIVE_CAM_X, DRIVE_CAM_Y); //Driver cam control

  //Timer Objects
  private Timer timer = new Timer();

  //Power Distribution Board
  public static PowerDistributionPanel pdu = new PowerDistributionPanel(0); //PDU on CANBUS 0

  //Pneumatics
  public static Compressor c = new Compressor(0); //Air Compressor
  AnalogInput airPressure = new AnalogInput(0); //Pressure readings

  //ALL INPUTS ARE INPUT PULLUP... TRUE == OFF
  DigitalInput switchArmTiltTop = new DigitalInput(0); // Limit Switch 1
  DigitalInput switchArmHeightTop_ArmMounted = new DigitalInput(1); // Limit Switch 2
  DigitalInput switchArmHeightBottom = new DigitalInput(2); // Limit Switch 3
  DigitalInput switchArmHeightTop_SideMounted = new DigitalInput(3);
  
  public static int egg = 0; //EDUCATION

  //Arm Control
  public static ArmControl arms = new ArmControl(PWM_ARM_HEIGHT, PWM_ARM_TILT, PWM_ARM_BALL, PCM_ARM_EJECT_ONE, PCM_ARM_EJECT_TWO); //Arm Control Object

  //HID Device Values ------------------------------------------------------------
  private double driveX;
  private double driveY;
  private double driveZ;
  private double armLeftX;
  private double armRightX;
  private double armLeftY;
  private double armRightY;
  private double armTriggerR;
  private double armTriggerL;
  private boolean armBumperL;
  
  //BEGIN ROBOT CODE --------------------------------------------------------------
  //-------------------------------------------------------------------------------------------
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
  //-------------------------------------------------------------------------------------------
  @Override
  public void autonomousInit()
  {
    timer.reset(); //Reset the timer
    timer.start(); //Sta                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   cccccccccccc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    x                                       rt the timer
    getHIDInputValues();
    updateSmartDashboard();
  }
  //-------------------------------------------------------------------------------------------
  @Override
  public void autonomousPeriodic()
  {
    //DON'T PUT THINGS IN HERE
    getHIDInputValues();
    updateSmartDashboard();
    robotActions();
    
  }
  //-------------------------------------------------------------------------------------------
  @Override
  public void teleopInit()
  {
    getHIDInputValues();
    updateSmartDashboard();
  }
  //-------------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic()
  { 
    getHIDInputValues();
    updateSmartDashboard();
    robotActions();
  }
  //-------------------------------------------------------------------------------------------
  @Override public void disabledInit()
  {    
    
  }
  //-------------------------------------------------------------------------------------------
  private void updateSmartDashboard() {
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
    SmartDashboard.putBoolean("Switch 1 (arm tilt)", switchArmTiltTop.get());
    SmartDashboard.putBoolean("Switch 2 (arm height top, arm mounted)", switchArmHeightTop_ArmMounted.get());
    SmartDashboard.putBoolean("Switch 3 (arm height bottom", switchArmHeightBottom.get());
    SmartDashboard.putBoolean("Switch 4 (arm height top, side mounted)", switchArmHeightTop_SideMounted.get());
    SmartDashboard.putNumber("ArmTriggerL", armTriggerL);
  }
  //------------------------------------------------------------------------------------------
  private void getHIDInputValues() {
    driveX = driveControl.getX(); //Get joystick x
    driveY = -driveControl.getY(); //Get joysitck y
    driveZ = driveControl.getZ(); //Get joystick z

    armLeftX = armControl.getX(Hand.kLeft); //Get xbox LS x
    armLeftY = -armControl.getY(Hand.kLeft); //Get xbox LS y
    
    armRightX = armControl.getX(Hand.kRight); //Get xbox RS x
    armRightY = -armControl.getY(Hand.kRight); //Get xbox RS y

    armTriggerL = armControl.getTriggerAxis(Hand.kLeft); //Get xbox LT val
    armTriggerR = armControl.getTriggerAxis(Hand.kRight); //Get xbox RT val
    
    armBumperL = armControl.getBumper(Hand.kLeft);

  }
  //------------------------------------------------------------------------------------------x 
  private void robotActions() {
    driverCam.look(driveControl.getPOV());
    //Move the robot
    robotDrive.driveCartesian(0.0, driveY, driveX);
    //robotDrive.driveCartesian(driveX, driveY, driveZ);

    //Move the arms
    if(armLeftY >= 0.15 && switchArmHeightTop_ArmMounted.get() || switchArmHeightTop_SideMounted.get()){
      if (switchArmHeightTop_ArmMounted.get() && switchArmHeightTop_SideMounted.get()) {
        //Full Speed, No switches pressed
        arms.moveArms(armLeftY);
      }else{
        // One switch pressed - we're close to the top. Slow way down
        arms.moveArms(armLeftY / 4);
      }
    } else if (armLeftY <= -0.15 && switchArmHeightBottom.get()) {
      arms.moveArms(armLeftY);
    } else {
      arms.moveArms(0.0);
    }

    //Tilt the arms
    if (armRightY < -0.15) {
      // must make armRight into a positive number to make it go down
      arms.tiltArms(-armRightY /2);
    } else if (armRightY > 0.15 && switchArmTiltTop.get()){
      // need to negate armRight in order to tilt arm up
      arms.tiltArms(-armRightY / 2);
    } else {
      arms.tiltArms(0.0);
    }

    //Eject ball
    if (armTriggerR > 0.25) {
      arms.ejectBall();
    } else if (armControl.getBumper(Hand.kRight)) {
      arms.fetchBall();
    } else {
      arms.stopBallMotors();
    }

     //Pancakes
     if (armBumperL == true) {
      arms.ejectPancakeExtend();
      System.out.println("pancake eject");
     //} else if (armControl.getYButton()){
       //arms.ejectPancakeRetract();
     } else {
       arms.ejectPancakeStop();
     }

    //Flip Ramp
    if(armControl.getBButton()) {
      rampMotor.set(1.0);
    } else if (armControl.getXButton()) {
      rampMotor.set(-1.0);
    } else {
      rampMotor.set(0.0);
    }
  }
  //-------------------------------------------------------------------------------------------
}

