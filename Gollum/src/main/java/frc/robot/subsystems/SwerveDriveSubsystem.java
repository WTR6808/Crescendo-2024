// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Sensors.FieldManagementSystem;
import frc.robot.Sensors.LimelightTwo;
import frc.robot.Sensors.PigeonTwo;

public class SwerveDriveSubsystem extends SubsystemBase {
  private static SwerveDriveSubsystem instance = null;

  //Create the four Swerve Modules
  private final MK4_L3_SwerveModule m_frontLeft =
    new MK4_L3_SwerveModule("Left Front", 
                            Constants.SwerveDriveConstants.FRONT_LEFT_DRIVE_CANID,
                            Constants.SwerveDriveConstants.FRONT_LEFT_DRIVE_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_LEFT_DRIVE_ENCODER_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_LEFT_TURN_CANID,
                            Constants.SwerveDriveConstants.FRONT_LEFT_TURN_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_LEFT_ENCODER_CAN_ID,
                            Constants.SwerveDriveConstants.FRONT_LEFT_OFFSET);         

  private final MK4_L3_SwerveModule m_frontRight =
    new MK4_L3_SwerveModule("Right Front", 
                            Constants.SwerveDriveConstants.FRONT_RIGHT_DRIVE_CANID,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_DRIVE_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_DRIVE_ENCODER_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_TURN_CANID,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_TURN_INVERTED,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_ENCODER_CAN_ID,
                            Constants.SwerveDriveConstants.FRONT_RIGHT_OFFSET);         

  private final MK4_L3_SwerveModule m_backRight =
    new MK4_L3_SwerveModule("Right Back", 
                            Constants.SwerveDriveConstants.BACK_RIGHT_DRIVE_CANID,
                            Constants.SwerveDriveConstants.BACK_RIGHT_DRIVE_INVERTED,
                            Constants.SwerveDriveConstants.BACK_RIGHT_DRIVE_ENCODER_INVERTED,
                            Constants.SwerveDriveConstants.BACK_RIGHT_TURN_CANID,
                            Constants.SwerveDriveConstants.BACK_RIGHT_TURN_INVERTED,
                            Constants.SwerveDriveConstants.BACK_RIGHT_ENCODER_CAN_ID,
                            Constants.SwerveDriveConstants.BACK_RIGHT_OFFSET); 

  private final MK4_L3_SwerveModule m_backLeft =
    new MK4_L3_SwerveModule("Left Back", 
                            Constants.SwerveDriveConstants.BACK_LEFT_DRIVE_CANID,
                            Constants.SwerveDriveConstants.BACK_LEFT_DRIVE_INVERTED,
                            Constants.SwerveDriveConstants.BACK_LEFT_DRIVE_ENCODER_INVERTED,
                            Constants.SwerveDriveConstants.BACK_LEFT_TURN_CANID,
                            Constants.SwerveDriveConstants.BACK_LEFT_TURN_INVERTED,
                            Constants.SwerveDriveConstants.BACK_LEFT_ENCODER_CAN_ID,
                            Constants.SwerveDriveConstants.BACK_LEFT_OFFSET);
                            
  //Get Instance CTRE Pigeon 2
  PigeonTwo m_pigeon = PigeonTwo.getInstance();

  //Get Instance of the Limelight
  LimelightTwo m_limelight = LimelightTwo.Instance();

  //Get Instance of Field Management System Information
  FieldManagementSystem FMSInfo = FieldManagementSystem.getInstance();

  //Used for current and voltage graphing;
  private Double current;
  private Double voltage;
  
  /** Creates a new SwerveDriveSubsystem. */
  public static SwerveDriveSubsystem Instance() {
    if (instance == null) 
      instance = new SwerveDriveSubsystem();

    return instance;
  }

  private SwerveDriveSubsystem() {
//    SmartDashboard.putNumber("Drive P", Constants.SDSModuleConstants.DRIVE_P);
//    SmartDashboard.putNumber("Drive I", Constants.SDSModuleConstants.DRIVE_I);
//    SmartDashboard.putNumber("Drive D", Constants.SDSModuleConstants.DRIVE_D);
//    SmartDashboard.putNumber("Turn P", Constants.SDSModuleConstants.TURN_P);
//    SmartDashboard.putNumber("Turn I", Constants.SDSModuleConstants.TURN_I);
//    SmartDashboard.putNumber("Turn D", Constants.SDSModuleConstants.TURN_D);
  }
  public void stop (){
    drive (0,0,0,false);
  }

  public void drive (double x, double y, double rot, boolean fieldRelative){
    ChassisSpeeds speeds;
    x = smoother_input(MathUtil.applyDeadband(x, Constants.OperatorConstants.DEADZONE));
    y = smoother_input(MathUtil.applyDeadband(y, Constants.OperatorConstants.DEADZONE));
    rot = smoother_input(MathUtil.applyDeadband(rot, Constants.OperatorConstants.DEADZONE));

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
    SwerveModuleState[] swerveModuleStates;
    if (Math.abs(x)   >= Constants.OperatorConstants.DEADZONE ||
        Math.abs(y)   >= Constants.OperatorConstants.DEADZONE ||
        Math.abs(rot) >= Constants.OperatorConstants.DEADZONE){
      swerveModuleStates = Constants.SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    } else {
      swerveModuleStates = new SwerveModuleState[]{
        new SwerveModuleState(0.0, new Rotation2d( Math.PI/4.0)), //LF
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4.0)), //RF
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4.0)), //LB
        new SwerveModuleState(0.0, new Rotation2d( Math.PI/4.0))  //RB
      };
    }
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

  //Drive encoder routines
  //Sets Drive Encoder Position to 0
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  //Returns the average of the Drive Encoder Distances in Meters
  public double getAvgDistance(){
    SmartDashboard.putNumber("Front Left Encoder: ", m_frontLeft.getDriveDistance());
    SmartDashboard.putNumber("Front Right Encoder: ", m_frontRight.getDriveDistance());
    SmartDashboard.putNumber("Back Left Encoder: ", m_backLeft.getDriveDistance());
    SmartDashboard.putNumber("Back Right Encoder: ", m_backRight.getDriveDistance());
    return ((m_frontLeft.getDriveDistance()+
             m_backLeft.getDriveDistance() +
             m_frontRight.getDriveDistance() +
             m_backRight.getDriveDistance()) / 4.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    current = (m_frontLeft.get_current() + m_frontRight.get_current() + m_backLeft.get_current() + m_backRight.get_current()) / 4;
    SmartDashboard.putNumber("Current", current);

    voltage = (m_frontLeft.get_voltage() + m_frontRight.get_voltage() + m_backLeft.get_voltage() + m_backRight.get_voltage()) / 4;
    SmartDashboard.putNumber("Voltage", voltage);

    SmartDashboard.putNumber("Gyro", m_pigeon.getAngle().getDegrees());
    m_limelight.periodic(true);
    
    //Added for debugging drive/strafe distance routines
    getAvgDistance();
  }

  //Formula by Sergey to smooth out the driving inputs
  private double smoother_input(double d) {
    if (d==0) {
      return 0;
    }
    return d<0 ? -((Math.pow(d, 2)+ Math.pow(d, 4)) / 2) : ((Math.pow(d, 2)+ Math.pow(d, 4)) / 2);
    //return Math.signum(d) * ((Math.pow(d, 2)+ Math.pow(d, 4)) / 2);
  }

  //Exposed Gyro Commands
  //Set the Heading to 0.0 degrees
  public void reset_pigeon2() {
    m_pigeon.reset();
  }

  //Set the Heading to the provided degrees
  public void setPigeon2Angle(double deg){
    m_pigeon.setAngle(deg);
  }

  public double getPigeonHeading(){
    return m_pigeon.get360Angle();
  }

  //Exposed Limelight Commands
  public boolean targetAcquired(){
    return m_limelight.targetAcquired();
  }

  public int targetID(){
    return m_limelight.getTagNumber();
  }

  public void takeSnapShot(){
    m_limelight.takeSnapShot();
  }

  private void initDriveToTarget(){
    this.stop();
    m_limelight.initializeTargetTracking();
  }
  public void InitDriveToSpeaker(){
    initDriveToTarget();
    m_limelight.setPipeline(Constants.Limelight_Constants.SPEAKER_PIPE);
  }

  public void InitAutoDriveToSpeaker(){
    initDriveToTarget();
    m_limelight.setPipeline(Constants.Limelight_Constants.SPEAKER_AUTO_PIPE);
  }
  public void InitDriveToAmp(){
    initDriveToTarget();
    m_limelight.setPipeline(Constants.Limelight_Constants.AMP_PIPE);
  }

 public void stopDriveToTarget(){
    this.stop();
    m_limelight.endTargetTracking();
  }

  public boolean driveToTarget(){
    boolean valid = true;
    /**********************************************************
      Call alternate calcMotorControl here
      May have to change fieldRelative to true

      May also have to use a different drive command that
      rotates the bot to 0, 90, 180 or 270
      sets an angle on the wheels and drives at that angle
      for a computed distance and then searches for target
      again to check errors in X and Y to adjust.
    ***********************************************************/
    if (m_limelight.calcMotorControl()){
      drive(m_limelight.driveCommand(), 
            m_limelight.strafeCommand(), 
            m_limelight.steerCommand(),
            false);
    } else {
      valid = false;
    }
    return valid;
  }
}
