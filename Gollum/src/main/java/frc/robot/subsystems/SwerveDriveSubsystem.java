// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Sensors.PigeonTwo;

public class SwerveDriveSubsystem extends SubsystemBase {
  private static SwerveDriveSubsystem instance = null;

    //Create the four Swerve Modules
  private final MK4_L3_SwerveModule m_frontLeft =
      new MK4_L3_SwerveModule("Left Front", 
                              Constants.SwerveDriveConstants.FRONT_LEFT_DRIVE_CANID,
                              Constants.SwerveDriveConstants.FRONT_LEFT_DRIVE_INVERTED,
                              Constants.SwerveDriveConstants.FRONT_LEFT_TURN_CANID,
                              Constants.SwerveDriveConstants.FRONT_LEFT_TURN_INVERTED,
                              Constants.SwerveDriveConstants.FRONT_LEFT_ENCODER_CAN_ID);         

  private final MK4_L3_SwerveModule m_frontRight =
      new MK4_L3_SwerveModule("Right Front", 
                              Constants.SwerveDriveConstants.FRONT_RIGHT_DRIVE_CANID,
                              Constants.SwerveDriveConstants.FRONT_RIGHT_DRIVE_INVERTED,
                              Constants.SwerveDriveConstants.FRONT_RIGHT_TURN_CANID,
                              Constants.SwerveDriveConstants.FRONT_RIGHT_TURN_INVERTED,
                              Constants.SwerveDriveConstants.FRONT_RIGHT_ENCODER_CAN_ID);         

  private final MK4_L3_SwerveModule m_backRight =
      new MK4_L3_SwerveModule("Right Back", 
                              Constants.SwerveDriveConstants.BACK_RIGHT_DRIVE_CANID,
                              Constants.SwerveDriveConstants.BACK_RIGHT_DRIVE_INVERTED,
                              Constants.SwerveDriveConstants.BACK_RIGHT_TURN_CANID,
                              Constants.SwerveDriveConstants.BACK_RIGHT_TURN_INVERTED,
                              Constants.SwerveDriveConstants.BACK_RIGHT_ENCODER_CAN_ID); 

  private final MK4_L3_SwerveModule m_backLeft =
      new MK4_L3_SwerveModule("Left Back", 
                              Constants.SwerveDriveConstants.BACK_LEFT_DRIVE_CANID,
                              Constants.SwerveDriveConstants.BACK_LEFT_DRIVE_INVERTED,
                              Constants.SwerveDriveConstants.BACK_LEFT_TURN_CANID,
                              Constants.SwerveDriveConstants.BACK_LEFT_TURN_INVERTED,
                              Constants.SwerveDriveConstants.BACK_LEFT_ENCODER_CAN_ID);
                              
  //Create CTRE Pigeon 2
  PigeonTwo m_pigeon = PigeonTwo.getInstance();

  /** Creates a new SwerveDriveSubsystem. */
  public static SwerveDriveSubsystem Instance() {
    if (instance == null) 
      instance = new SwerveDriveSubsystem();

    return instance;
  }
  private SwerveDriveSubsystem() {}

  public void drive (double x, double y, double rot, boolean fieldRelative){
    ChassisSpeeds speeds;
    x = MathUtil.applyDeadband(x, Constants.OperatorConstants.DEADZONE);
    y = MathUtil.applyDeadband(y, Constants.OperatorConstants.DEADZONE);
    rot = MathUtil.applyDeadband(rot, Constants.OperatorConstants.DEADZONE);

    //TODO Apply deadband to x, y, and rot
    if (fieldRelative)
    {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, m_pigeon.getAngle());
    }
    else
    {
      speeds = new ChassisSpeeds(x,y,rot);
    }
    //Output x,y, rot and Chassis Speeds to SmartDashboard
    SmartDashboard.putNumber("Input x: ", x);
    SmartDashboard.putNumber("Input y: ", y);
    SmartDashboard.putNumber("Input r: ", rot);
    SmartDashboard.putNumber("Speed x: ", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Speed y: ", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Omega  : ", speeds.omegaRadiansPerSecond);
    var swerveModuleStates = Constants.SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    //TODO replace swerveModuleStates with ...
    //var swerveModuleStates = Constants.SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
    //                                ChassisSpeeds.discretize(speeds, TimedRobot.kDefaultPeriod));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 
                                                Constants.Measurements.ROBOT_MAX_LINEAR_VELOCITY);
    m_frontLeft.setDesiredState  (swerveModuleStates[0]);                                                
    m_frontRight.setDesiredState (swerveModuleStates[1]);                                                
    m_backLeft.setDesiredState   (swerveModuleStates[2]);                                                
    m_backRight.setDesiredState  (swerveModuleStates[3]);                                                
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
