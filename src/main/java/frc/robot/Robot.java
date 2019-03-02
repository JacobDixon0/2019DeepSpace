/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

  import edu.wpi.first.cameraserver.CameraServer;
  import edu.wpi.first.wpilibj.DigitalInput;
  import edu.wpi.first.wpilibj.DoubleSolenoid;
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.PWMTalonSRX;
  import edu.wpi.first.wpilibj.RobotDrive;
  import edu.wpi.first.wpilibj.Spark;
  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
  import edu.wpi.first.wpilibj.drive.MecanumDrive;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
  import io.github.pseudoresonance.pixy2api.Pixy2;
  import io.github.pseudoresonance.pixy2api.links.I2CLink;
  import frc.robot.state_machine.*;
  

  import frc.robot.RobotMap;

  //import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
  

public class Robot extends TimedRobot {

  // Controls 
  private Joystick m_stick;
  private Joystick m_gamePad;

  // Robot
  private MecanumDrive m_robotDrive;
  
  // End effector
  private DoubleSolenoid m_bumperReach;
  private DoubleSolenoid m_kicker;
  
  // Climber
  private DoubleSolenoid m_frontSolenoid;
  private DoubleSolenoid m_backSolenoid;
  
  // Elevator
  //private WPI_TalonSRX m_elevatorLeft;
  //private WPI_TalonSRX m_elevatorRight;
  private Spark m_elevatorLeft;
  private Spark m_elevatorRight;

  private DigitalInput m_limitSwitchBottom;
  private DigitalInput m_limitSwitchTop;
  private enum ElevatorDirection {STOPPED, UP, DOWN};
  private ElevatorDirection elevatorPosition = ElevatorDirection.STOPPED;
  
  // Camera
  private Pixy2 m_pixy2;
  
  // State
  private boolean m_startClimb = false; 
  
  
  // Autonomous
  private StateMachine sm;


  private StateMachine buildStateMachine() {
    StateMachine sm = new StateMachine();

    // Lift the front of the robot onto the platform
    sm.addState(new StateDriveY(m_robotDrive, -0.4, 1.0));
    sm.addState(new StateSoliniod(m_frontSolenoid, Value.kForward, 1.0));
    sm.addState(new StateDriveY(m_robotDrive, 0.4, 1.0));

    // Retract front and extend back (lifting back of robot onto platform)
    StateGroup sg = new StateGroup();
    sg.addState(new StateSoliniod(m_frontSolenoid, Value.kReverse, 1.0));
    sg.addState(new StateSoliniod(m_backSolenoid, Value.kForward, 1.0));
    sm.addState(sg);

    // Get back of robot onto platform
    sm.addState(new StateDriveY(m_robotDrive, 0.4, 1.0));
    sm.addState(new StateSoliniod(m_backSolenoid, Value.kForward, 1.0));

    return sm;
  }

  private double squareInput(double x) {
    if (x < 0) {
      x = -x*x;
    }
    else {
      x = x*x;
    }
    return x;
  }


  @Override
  public void robotInit() {




    //CameraServer.getInstance().startAutomaticCapture();
    // Inputs
    m_stick = new Joystick(RobotMap.kJoystick);
    m_gamePad = new Joystick(RobotMap.kGamepad);
    //spinJoystick = new Joystick(2);
    //liftMotor = new WPI_TalonSRX(liftMotorChannel);

    //m_gyro = new AnalogGyro(0);
    
    // Pneumatics
    m_bumperReach = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kBumperReachForward,RobotMap.kBumperReachReverse);
    m_kicker = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kKickerForward,RobotMap.kKickerReverse);
    //m_lock = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kLockForward,RobotMap.kLockReverse);
    m_frontSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kClimberFrontForward,RobotMap.kClimberFrontReverse);
    m_backSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kClimberBackForwards,RobotMap.kClimberBackReverse);
   
    // Drive Terrain
    //WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.kFrontLeftChannel);
    //WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.kRearLeftChannel);
    //WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.kFrontRightChannel);
    //WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.kRearRightChannel);
    Spark frontLeft = new Spark(0);
    Spark frontRight = new Spark(1);
    Spark rearLeft = new Spark(2);
    Spark rearRight = new Spark(3);
  
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // Elevator 
  
    //m_elevatorLeft = new WPI_TalonSRX(RobotMap.kElevatorLeft); // swich pwm to WPI)_ 
    //m_elevatorRight = new WPI_TalonSRX(RobotMap.kElevatorRight);
    Spark m_elevatorLeft = new Spark(4);
    Spark m_elevatorRight = new Spark(5);
    
    m_elevatorRight.setInverted(true);
    m_limitSwitchBottom = new DigitalInput(RobotMap.kLimitSwitchBottom);
    m_limitSwitchTop = new DigitalInput(RobotMap.kLimitSwitchTop);

    //Pixy2 pixy2 = Pixy2.createInstance(Pixy2.LinkType.I2C);
    //pixy2.init();
    //Testing pixy2 LED Color

    sm = buildStateMachine();

  }

  @Override
  public void teleopInit(){
    m_bumperReach.set(DoubleSolenoid.Value.kReverse);
    m_frontSolenoid.set(DoubleSolenoid.Value.kReverse);
    //backSolenoid.set(DoubleSolenoid.Value.kReverse);
   
  }

  @Override
  public void teleopPeriodic() {

    ///////////////////////////////////////////////////////////////////////////
    // Driving Code
    double speed = 0.0;
    if (m_stick.getRawButton(1)) {
      speed = 1.0; // Overddrive (press trigger)
    } else {
      speed = 0.70; // Normal case (70%)
    }

    m_robotDrive.driveCartesian(-speed*squareInput(m_stick.getX()),
                                speed*squareInput(m_stick.getY()),
                                -speed*squareInput(m_stick.getThrottle()));


    ///////////////////////////////////////////////////////////////////////////
    // Elevator
    
    //TODO put on to POV

    if (m_gamePad.getRawButtonPressed(1)) {
      elevatorPosition = ElevatorDirection.DOWN;
    } else if (m_gamePad.getRawButtonPressed(2)) {
      elevatorPosition = ElevatorDirection.UP;
    } 

    // lower end stop
    if (m_limitSwitchBottom.get() && elevatorPosition == ElevatorDirection.DOWN) {
      elevatorPosition = ElevatorDirection.STOPPED;
    }

    // upper end stop
    if (m_limitSwitchTop.get() && elevatorPosition == ElevatorDirection.UP) {
      elevatorPosition = ElevatorDirection.STOPPED;
    }

    if (elevatorPosition == ElevatorDirection.UP) {
      m_elevatorLeft.set(0.7);
      m_elevatorRight.set(0.7);
    } else if (elevatorPosition == ElevatorDirection.DOWN) {
      m_elevatorLeft.set(-0.7);
      m_elevatorRight.set(-0.7);
    }


     //limit switchs
    /*
    }*/
/*
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
    */
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

     SmartDashboard.putNumber("POV", m_gamePad.getPOV());

    if (m_gamePad.getRawButton(8) && m_gamePad.getRawButtonPressed(2)) {
      m_startClimb = true;      
    }

    if (m_gamePad.getRawButton(1)) { 
      m_startClimb = false;
      m_backSolenoid.set(Value.kReverse);
      m_frontSolenoid.set(Value.kReverse);
    }
    
    SmartDashboard.putBoolean("m_startClimb", m_startClimb);

    if (sm.isDone()){
      m_startClimb = false;
    }

    if (m_startClimb){
      //sm.execute();
    }
    else{
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





