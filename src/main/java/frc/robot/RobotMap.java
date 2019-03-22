/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotMap {
    
    // USB inputs
    public static final int kJoystick = 0;
    public static final int kGamepad = 1;

    // CAN
    public static final int kFrontLeftChannel = 21;
    public static final int kRearLeftChannel = 22;
    public static final int kFrontRightChannel = 20;
    public static final int kRearRightChannel = 23;
    public static final int kElevatorLeft = 24; // set to 24
    public static final int kElevatorRight = 25;

    // Digital Inputs
    public static final int kLimitSwitchBottom = 0;
    public static final int kLimitSwitchTop = 1;

    // Pneumatic Module
    public static final int kSolenoidModule = 2;
    public static final int kBumperReachForward = 6;
    public static final int kBumperReachReverse = 7;
    public static final int kKickerForward = 2;
    public static final int kKickerReverse = 3;
    /*public static final int kLockReverse = 0;
    public static final int kLockForward = 1;*/
    public static final int kClimberFrontForward = 0;
    public static final int kClimberFrontReverse = 1;
    public static final int kClimberBackForwards = 4;
    public static final int kClimberBackReverse = 5;

  
    //public static final int liftMotorChannel = 4;

}
