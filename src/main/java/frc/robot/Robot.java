/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.state_machine.*;
import frc.robot.Pixy2Handler;
import frc.robot.RobotMap;
import frc.robot.LineFollowing;

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
  private WPI_TalonSRX m_elevatorLeft;
  private WPI_TalonSRX m_elevatorRight;
  // private Spark m_elevatorLeft;
  // private Spark m_elevatorRight;

  private DigitalInput m_limitSwitchBottom;
  private DigitalInput m_limitSwitchTop;

  // Camera
  private Pixy2Handler m_pixy2;
  private DigitalInput m_lineFind;
  private LineFollowing m_lineFollower;

  // State
  private boolean m_startClimb = false;

  // Autonomous
  private StateMachine sm;

 

  private StateMachine buildStateMachine() {
    StateMachine sm = new StateMachine("ClimbMachine");

    // Lift the front of the robot onto the platform
    sm.addState(new StateSolenoid(m_frontSolenoid, Value.kForward, 1.0));
    sm.addState(new StateDriveY(m_robotDrive, 0.4, 2.0));

    // Retract front and extend back (lifting back of robot onto platform)
    StateGroup sg = new StateGroup("LiftGroup");
    sg.addState(new StateSolenoid(m_frontSolenoid, Value.kReverse, 2.0));
    sg.addState(new StateSolenoid(m_backSolenoid, Value.kForward, 2.0));
    sm.addState(sg);

    // Get back of robot onto platform
    sm.addState(new StateDriveY(m_robotDrive, 0.4, 2.0));
    sm.addState(new StateSolenoid(m_backSolenoid, Value.kForward, 1.0));
    sm.addState(new StateDriveY(m_robotDrive, 0.4, 2.0));

    return sm;
  }

  private double squareInput(double x) {
    if (x < 0) {
      x = -x * x;
    } else {
      x = x * x;
    }
    return x;
  }

  private void elevator(double y){
    m_elevatorRight.set(y);
    m_elevatorLeft.set(y);
  }

  private void climb (Joystick j, VictorSPX left, VictorSPX right) {
    left.setInverted(true);
    right.setInverted(false);

  }

  @Override
  public void robotInit() {

    m_pixy2 = new Pixy2Handler();
    m_pixy2.init();
    m_lineFind = new DigitalInput(2);
    m_lineFollower = new LineFollowing(m_pixy2);
    // Pixy2 m_pixy2 = Pixy2.createInstance(Pixy2.LinkType.I2C);
    
    // Testing pixy2 LED Color


    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);

    // Inputs
    m_stick = new Joystick(RobotMap.kJoystick);
    m_gamePad = new Joystick(RobotMap.kGamepad);

    // spinJoystick = new Joystick(2);
    // liftMotor = new WPI_TalonSRX(liftMotorChannel);

    // m_gyro = new AnalogGyro(0);

    // Pneumatics
    m_bumperReach = new DoubleSolenoid(RobotMap.kSolenoidModule, RobotMap.kBumperReachForward,
        RobotMap.kBumperReachReverse);
    m_kicker = new DoubleSolenoid(RobotMap.kSolenoidModule, RobotMap.kKickerForward, RobotMap.kKickerReverse);
    // m_lock = new
    // DoubleSolenoid(RobotMap.kSolenoidModule,RobotMap.kLockForward,RobotMap.kLockReverse);
    m_frontSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule, RobotMap.kClimberFrontForward,
        RobotMap.kClimberFrontReverse);
    m_backSolenoid = new DoubleSolenoid(RobotMap.kSolenoidModule, RobotMap.kClimberBackForwards,
        RobotMap.kClimberBackReverse);

    // Drive Terrain
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(RobotMap.kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(RobotMap.kRearRightChannel);
    // Spark frontLeft = new Spark(0);
    // Spark frontRight = new Spark(1);
    // Spark rearLeft = new Spark(2);
    // Spark rearRight = new Spark(3);

    VictorSPX climbLeft = new VictorSPX(RobotMap.kClimbLeft);
    VictorSPX climbRight = new VictorSPX(RobotMap.kClimbRight);

    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // Elevator

    m_elevatorLeft = new WPI_TalonSRX(RobotMap.kElevatorLeft); // swich pwm to WPI)_
    m_elevatorRight = new WPI_TalonSRX(RobotMap.kElevatorRight);
    // Spark m_elevatorLeft = new Spark(4);
    // Spark m_elevatorRight = new Spark(5);

    m_elevatorRight.setInverted(true);
    m_limitSwitchBottom = new DigitalInput(RobotMap.kLimitSwitchBottom);
    m_limitSwitchTop = new DigitalInput(RobotMap.kLimitSwitchTop);

    sm = buildStateMachine();
  }

  @Override
  public void autonomousInit() {
    teleopInit();
  }

  @Override
  public void teleopInit() {
    m_bumperReach.set(DoubleSolenoid.Value.kReverse);
    m_frontSolenoid.set(DoubleSolenoid.Value.kReverse);
    m_backSolenoid.set(DoubleSolenoid.Value.kReverse);
    if (!m_pixy2.lampOn){
      m_pixy2.toggleLamp();
    }
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopPeriodic() {


    m_pixy2.sendRequest(m_pixy2.CHECKSUM_GETMAINFEATURES);
    //m_pixy2.HandleInput();
    // m_pixy2.printLocalCache();

    ///////////////////////////////////////////////////////////////////////////
    // Driving Code

    double speed = 0.0;
    if (m_stick.getRawButton(1)) {
      speed = 1.0; // Overddrive (press trigger)
    } else {
      speed = 0.70; // Normal case (70%)
    }

    //double x = squareInput(m_stick.getX());
    //double y = squareInput(m_stick.getY());
    //double z = squareInput(m_stick.getThrottle());

    double x = m_stick.getX();
    double y = m_stick.getY();
    double z = m_stick.getThrottle();

    SmartDashboard.putBoolean("Locked On", m_lineFollower.lineIsValid());
    SmartDashboard.putNumber("sidle", m_lineFollower.getSidleError());
    SmartDashboard.putNumber("Angle", m_lineFollower.getAngleError());
    
    SmartDashboard.putNumber("get x",m_lineFollower.getX());
    SmartDashboard.putNumber("get z",m_lineFollower.getZ());

    if (m_stick.getRawButton(4) && m_lineFollower.lineIsValid()) {
      m_robotDrive.driveCartesian(m_lineFollower.getX(), speed * y, m_lineFollower.getZ());
    } else {
        m_robotDrive.driveCartesian(-speed * x, speed * y, -speed * z);  
    }
    // manual climb

    // Back Solenoid
    if (m_stick.getRawButtonPressed(5)) {
      if (m_backSolenoid.get() == Value.kReverse) {
        m_backSolenoid.set(DoubleSolenoid.Value.kForward);
      } else if (m_backSolenoid.get() == Value.kForward) {
        m_backSolenoid.set(DoubleSolenoid.Value.kReverse);
      } else if (m_backSolenoid.get() == Value.kOff) {
        m_backSolenoid.set(DoubleSolenoid.Value.kForward);
      }
    }
    // front Solenoid
    if (m_stick.getRawButtonPressed(6)) {
      if (m_frontSolenoid.get() == Value.kReverse) {
        m_frontSolenoid.set(DoubleSolenoid.Value.kForward);
      } else if (m_frontSolenoid.get() == Value.kForward) {
        m_frontSolenoid.set(DoubleSolenoid.Value.kReverse);
      } else if (m_frontSolenoid.get() == Value.kOff) {
        m_frontSolenoid.set(DoubleSolenoid.Value.kForward);
      }
    }
    

    ///////////////////////////////////////////////////////////////////////////
    // Elevator

    //double boost = m_gamePad.getRawAxis(3);
    //double eSpeed = ( 0.5 * boost ) + 0.5;
    double elevatorQuiescentSpeed = 0.05;
    double elevatorDeadzone = 0.07;

    double elevatorBoost = (m_gamePad.getRawAxis(3) >= 0.5) ? 1.0 : 0.7; 
    double elevatorCommandedSpeed = elevatorBoost * -m_gamePad.getRawAxis(1);
    
    // upperLimitSwitch
    if (!m_limitSwitchTop.get() && elevatorCommandedSpeed > 0) {
      // hold the elevator up
      elevatorCommandedSpeed = elevatorQuiescentSpeed;
    }

    // lowerLimitSwitch
    if (!m_limitSwitchBottom.get() && elevatorCommandedSpeed < 0) {
      // hold the elevator up
      elevatorCommandedSpeed = elevatorQuiescentSpeed;
    }

    // Quiescent speed
    if (Math.abs(m_gamePad.getRawAxis(1)) < elevatorDeadzone) {
      elevatorCommandedSpeed = elevatorQuiescentSpeed;
    }

    elevator(elevatorCommandedSpeed);


    /*
    if (m_gamePad.getRawAxis(1) > 0 && !m_limitSwitchBottom.get()) {
      elevator(0.07);
      eSpeed = 0.0;
    } else if (m_gamePad.getRawAxis(1) < 0 && !m_limitSwitchTop.get()) {
      elevator(0.07);
      eSpeed = 0.0;
    } //else if (!m_limitSwitchBottom.get()){
      //elevator(0.07);
      //eSpeed = 0.0;
    //} 
    else{
      if (Math.abs(m_gamePad.getRawAxis(1)) < eDeadzone){
        elevator(0.07);
      } else {
        elevator(-m_gamePad.getRawAxis(1) * eSpeed);
      }
    }
    */

    
    // SmartDashboard.putBoolean("top", !m_limitSwitchTop.get());
    // SmartDashboard.putBoolean("Bottem", m_limitSwitchBottom.get());


    ///////////////////////////////////////////////////////////////////////////

    // Pixy2

    
    ///////////////////////////////////////////////////////////////////////////

    // Extender
    if (m_gamePad.getRawButtonPressed(6)) {
      
      // Unusual case - put here just to cover the corner case
      if (m_bumperReach.get() == DoubleSolenoid.Value.kOff) {
        m_bumperReach.set(DoubleSolenoid.Value.kOff);
      }

      if (m_bumperReach.get() == DoubleSolenoid.Value.kForward) {
        m_bumperReach.set(DoubleSolenoid.Value.kReverse);
      }

      if (m_bumperReach.get() == DoubleSolenoid.Value.kReverse) {
        m_bumperReach.set(DoubleSolenoid.Value.kForward);
      }
    }

    // The kicker

    if (m_gamePad.getRawButton(5)) {
      m_kicker.set(DoubleSolenoid.Value.kForward);
    } else {
      m_kicker.set(DoubleSolenoid.Value.kReverse);
    }

    // SmartDashboard.putBoolean("getRawButton(8)", m_gamePad.getRawButton(8));
    // SmartDashboard.putBoolean("getRawButton(2)", m_gamePad.getRawButton(2));

    if (m_gamePad.getRawButton(8) && m_gamePad.getRawButton(2)) {
      m_startClimb = true;
    }

    if (m_gamePad.getRawButton(1)) {
      m_startClimb = false;
      m_backSolenoid.set(Value.kReverse);
      m_frontSolenoid.set(Value.kReverse);
    }

    ///////////////////////////////////////////////////////////////////////////

    // SmartDashboard.putBoolean("m_startClimb", m_startClimb);
    // SmartDashboard.putBoolean("sm.isDone", sm.isDone());

    if (sm.isDone()) {
      m_startClimb = false;
    }

    if (m_startClimb) {
      sm.execute();
    } else {
      sm.reset();
    }

    // SmartDashboard.putString("Current State", sm.getName());

  }
  
  @Override
  public void disabledInit() {
    if (m_pixy2.lampOn) {
      m_pixy2.toggleLamp();
    }
  }
}
//JL was here