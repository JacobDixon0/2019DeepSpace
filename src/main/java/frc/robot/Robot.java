/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

  import edu.wpi.first.cameraserver.CameraServer;
  import edu.wpi.first.wpilibj.AnalogGyro;
  import edu.wpi.first.wpilibj.DigitalInput;
  import edu.wpi.first.wpilibj.DoubleSolenoid;
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.PWMTalonSRX;
  import edu.wpi.first.wpilibj.RobotDrive;
  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.Timer;
  import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
  import edu.wpi.first.wpilibj.drive.MecanumDrive;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
  import io.github.pseudoresonance.pixy2api.Pixy2;
  import io.github.pseudoresonance.pixy2api.links.I2CLink;
  import frc.robot.state_machine.StateMachine;
  import frc.robot.state_machine.StateWait;
  

  import frc.robot.RobotMap;

  import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
  

public class Robot extends TimedRobot {

  // Controls 
  private Joystick m_stick;
  private Joystick m_gamePad;
  private Joystick spinJoystick;

  // Robot
  private MecanumDrive m_robotDrive;
  private AnalogGyro m_gyro;
  private Timer m_timer;
  
  // End effector
  private DoubleSolenoid m_bumperReach;
  private DoubleSolenoid m_kicker;
  private DoubleSolenoid m_lock;
  
  // Climber
  private DoubleSolenoid m_frontSolenoid;
  private DoubleSolenoid m_backSolenoid;
  
  // Elevator
  private WPI_TalonSRX m_elevatorLeft;
  private WPI_TalonSRX m_elevatorRight;
  private DigitalInput m_limitSwitch;

  // Camera
  private Pixy2 m_pixy2;
  
  // State
  private boolean m_startClimb = false; 
  private boolean half;
  private boolean specialHalf;
  private double speed;
  
  // Autonomous
  private StateMachine sm;

  @Override
  public void robotInit() {




    CameraServer.getInstance().startAutomaticCapture();
    // Inputs
    m_stick = new Joystick(RobotMap.kJoystick);
    m_gamePad = new Joystick(RobotMap.kGamepad);
    //spinJoystick = new Joystick(2);
    //liftMotor = new WPI_TalonSRX(liftMotorChannel);

    //m_gyro = new AnalogGyro(0);
    m_timer = new Timer();
    
    // Pneumatics
    m_bumperReach = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kBumperReachForward,RobotMap.kBumperReachReverse);
    m_kicker = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kKickerForward,RobotMap.kKickerReverse);
    //m_lock = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kLockForward,RobotMap.kLockReverse);
    m_frontSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kClimberFrontForward,RobotMap.kClimberFrontReverse);
    m_backSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kClimberBackForwards,RobotMap.kClimberBackReverse);
   
    // Drive Terrain
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.kRearRightChannel);
  
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // Elevator
    m_elevatorLeft = new WPI_TalonSRX(RobotMap.kElevatorLeft);
    m_elevatorRight = new WPI_TalonSRX(RobotMap.kElevatorRight);
    m_elevatorRight.setInverted(true);
    m_limitSwitch = new DigitalInput(RobotMap.kLimitSwitch);

    //Pixy2 pixy2 = Pixy2.createInstance(Pixy2.LinkType.I2C);
    //pixy2.init();
    //Testing pixy2 LED Color

    sm.addState(new StateWait(1));
    sm.addState(new StateWait(2));

  }

  @Override
  public void teleopInit(){
    m_bumperReach.set(DoubleSolenoid.Value.kReverse);
    m_frontSolenoid.set(DoubleSolenoid.Value.kReverse);
    //backSolenoid.set(DoubleSolenoid.Value.kReverse);
    half = false;
    specialHalf = false;
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.

    
    double speed = 0.0;
    if (m_stick.getRawButton(1)) {
      speed = 1.0; // Overddrive (press trigger)
    } else {
      speed = 0.70; // Normal case (70%)
    }


    double x = m_stick.getX();
    if (x < 0) {
      x = -x*x;
    }
    else {
      x = x*x;
    }


    m_robotDrive.driveCartesian(-speed*x,
                                speed*m_stick.getY(),
                                -speed*m_stick.getThrottle());



/*
    // two joystick
    m_robotDrive.driveCartesian(-0.5*m_stick.getX(), 0.5*m_stick.getY(), -0.5*spinJoystick.getX());
    


*/
    // TEST
    SmartDashboard.putNumber("speed?", m_stick.getTwist()*-0.5+0.5);
    
      
    
     if (m_gamePad.getRawButton(1)) {
      m_elevatorLeft.set(0.7);
      m_elevatorRight.set(0.7);
    }
    else if (m_gamePad.getRawButton(2)) {
      m_elevatorLeft.set(-0.7);
      m_elevatorRight.set(-0.7);
    }
    else {
      m_elevatorLeft.set(0.0);
      m_elevatorRight.set(0.0);
    }
    
    // Extender
    if (m_gamePad.getRawButtonPressed(4)) {
      
      // Unusual case - put here just to cover the corner case
      if (m_bumperReach.get() == DoubleSolenoid.Value.kOff) {
        m_bumperReach.set(DoubleSolenoid.Value.kOff);
      }
      
      if (m_bumperReach.get() == DoubleSolenoid.Value.kForward) {
        m_bumperReach.set(DoubleSolenoid.Value.kReverse);
      }
      
      if (m_bumperReach.get() == DoubleSolenoid.Value.kReverse)  {
        m_bumperReach.set(DoubleSolenoid.Value.kForward);
      }
    }
   
     //The kicker
    
     if (m_gamePad.getRawButton(3))   {
       m_kicker.set(DoubleSolenoid.Value.kForward);
     }else{
       m_kicker.set(DoubleSolenoid.Value.kReverse);
     }



    if (m_gamePad.getRawButton(4)) {
      sm.execute();  
    }
    if (m_gamePad.getRawButtonReleased(4)) {
      sm.reset();
    }
        
    /*
    



    // PIXY2 DEBUG SECTION
    
  // SmartDashboard.putNumber("resoultion", pixy2.getResolution());


    // END PIXY2 DEBUG SECTION


    boolean gyroReset = m_gamePad.getRawButton(7);
    if(gyroReset){
      m_gyro.reset();
      System.out.println("Reset!");
    }
    
    
   
    //limit switchs
    /*if (!limitSwitch.get()){
      m_robotDrive.driveCartesian(1, 0, 0);
      System.out.println("LIMIT");
    }*/

    //Time Variables (1.0 IS PLACEHOLDER TIME FOR NOW) 
    /*
    double extendFront = 1.0;
    double extendBack = extendFront + 1.0;
    double retractBack = extendBack + 1.0;
    double finish = retractBack + 1.0;
   
    //Climbing Mechanism
  
      timer.reset();
      timer.start();
      m_robotDrive.driveCartesian(-1, 0, 0);
    
      if (timer.get() == extendFront){
        m_robotDrive.driveCartesian(0, 0, 0);
        frontSolenoid.set(DoubleSolenoid.Value.kForward);
        m_robotDrive.driveCartesian(1, 0, 0);
      
      }
     if (timer.get() == extendBack){
        m_robotDrive.driveCartesian(0, 0, 0);
        //backSolenoid.set(DoubleSolenoid.Value.kForward);
        frontSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_robotDrive.drive\Cartesian(1, 0, 0);
      
      }   
      if (timer.get() == retractBack){
        //backSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_robotDrive.driveCartesian(1, 0, 0);
      
      }
      if (timer.get() == finish){
        m_robotDrive.driveCartesian(0, 0, 0);
      }
    }*/
  }     
}




