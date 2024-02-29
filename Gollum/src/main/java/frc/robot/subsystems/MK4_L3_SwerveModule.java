// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MK4_L3_SwerveModule extends SubsystemBase {
  //Create Class Global Variables
  private final String    m_name;
  private CANSparkMax     m_driveMotor;
  private RelativeEncoder m_driveEncoder;
  private CANSparkMax     m_turnMotor;
  private CANcoder m_turnEncoder;

  //FIXME Change to on SparkMax PID Controller?
  /*private final PIDController m_drivePID = new PIDController(Constants.SDSModuleConstants.DRIVE_P,
                                                             Constants.SDSModuleConstants.DRIVE_I,
                                                             Constants.SDSModuleConstants.DRIVE_D); */
  //  private SparkPIDController m_drivePID;/*= new SparkPIDController(Constants.SDSModuleConstants.DRIVE_P,
  //                                                                Constants.SDSModuleConstants.DRIVE_I,
  //                                                                Constants.SDSModuleConstants.DRIVE_D);
  
  //FIXME Change to Profiled PID Controller after initial tuning?
  private final PIDController m_turnPID = new PIDController(Constants.SDSModuleConstants.TURN_P,
                                                            Constants.SDSModuleConstants.TURN_I,
                                                            Constants.SDSModuleConstants.TURN_D);

  /** Creates a new MK4_L3_SwerveModule. */
  public MK4_L3_SwerveModule(String name,
                             int driveCanID,
                             boolean driveInverted,
                             boolean driveEncoderInverted,
                             int turnCanID,
                             boolean turnInverted,
                             int turnEncoderID,
                             double turnEncoderOffset) {
    m_name = name;
    m_driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(driveInverted);
    m_driveMotor.setOpenLoopRampRate(0.5);
    //FIXME m_driveMotor.setSmartCurrentLimit(xx);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Constants.SDSModuleConstants.DRIVE_DISTANCE_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(Constants.SDSModuleConstants.DRIVE_VELOCITY_CONVERSION);
    m_driveEncoder.setInverted(driveEncoderInverted);

    m_turnMotor = new CANSparkMax(turnCanID, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setIdleMode(IdleMode.kBrake);
    m_turnMotor.setInverted(turnInverted);
    //FIXME m_turnMotor.setSmartCurrentLimit(xx);
    m_turnEncoder = new CANcoder(turnEncoderID);
    //m_turnEncoder.setVelocityConversionFactor(Constants.SDSModuleConstants.DRIVE_VELOCITY_CONVERSION);
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    //CANcoder turn config things
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;// Unsigned_0To1;
    configs.MagnetSensor.MagnetOffset = turnEncoderOffset; //the constant
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // Write these configs to the CANcoder
    m_turnEncoder.getConfigurator().apply(configs);
    //m_turnEncoder.setPosition(0);

    //Zero the Encoders
    resetEncoders();
  }

  public void resetEncoders(){
    m_driveEncoder.setPosition(0.0);
    //m_turnEncoder.setPosition(0.0);
  }

  public double getDriveDistance(){
    return m_driveEncoder.getPosition();
  }

  //TODO Replace this routine for Actual Swerve
  public void setDesiredState(SwerveModuleState desiredState){
    double driveOutput, turnOutput;
    //Rotation2d encoderRotation = new Rotation2d(m_turnEncoder.getPosition());
    Rotation2d encoderRotation = new Rotation2d(m_turnEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI); //now gets value between -pi & pi
    double driveVelocity = m_driveEncoder.getVelocity();

    SwerveModuleState state = SwerveModuleState.optimize(desiredState,encoderRotation);
    //driveOutput = m_drivePID.calculate(driveVelocity,state.speedMetersPerSecond);
    //driveOutput = driveOutput / Constants.Measurements.ROBOT_MAX_LINEAR_VELOCITY;
    driveOutput = state.speedMetersPerSecond;
    driveOutput = MathUtil.clamp(driveOutput, -.85, .85);


    turnOutput = MathUtil.clamp(m_turnPID.calculate(encoderRotation.getRadians(),state.angle.getRadians()),-1.0,1.0);

    //Send States to SmartDashboard
    SmartDashboard.putNumber(m_name+" Absolute Pos: ", m_turnEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber(m_name+" Desired Speed: ", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name+" Desired Angle: ", desiredState.angle.getDegrees());
    SmartDashboard.putNumber(m_name+" Optimized Speed:", state.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name+" Optimized Angle: ", state.angle.getRadians());

    SmartDashboard.putNumber(m_name+" Drive Encoder Vel: ", driveVelocity);
    SmartDashboard.putNumber(m_name+" Turn Encoder Pos: ", encoderRotation.getRadians());
    //SmartDashboard.putNumber(m_name+" Drive Output: ", driveOutput);
    SmartDashboard.putNumber(m_name+" Turn Output: ", turnOutput);
    //SmartDashboard.putNumber(m_name+" Desired State: ", driveOutput);
    m_driveMotor.set(driveOutput);
    m_turnMotor.set(turnOutput);
    
    //m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//    double dp,di,dd, tp, ti, td;
//    dp = SmartDashboard.getNumber("Drive P", Constants.SDSModuleConstants.DRIVE_P);
//    di = SmartDashboard.getNumber("Drive I", Constants.SDSModuleConstants.DRIVE_I);
//    dd = SmartDashboard.getNumber("Drive D", Constants.SDSModuleConstants.DRIVE_D);
//    tp = SmartDashboard.getNumber("Turn P", Constants.SDSModuleConstants.TURN_P);
//    ti = SmartDashboard.getNumber("Turn I", Constants.SDSModuleConstants.TURN_I);
//    td = SmartDashboard.getNumber("Turn D", Constants.SDSModuleConstants.TURN_D);

   /*  if (dp != m_drivePID.getP() || di != m_drivePID.getI() || dd != m_drivePID.getD()){
      m_drivePID.setPID(dp,di,dd);
    }

    if (tp != m_turnPID.getP() || ti != m_turnPID.getI() || td != m_turnPID.getD()){
      m_turnPID.setPID(tp,ti,td);
    }*/

    
  }

  public double get_current() {
    return m_driveMotor.getOutputCurrent();
  }

  public double get_voltage() {
    return m_turnMotor.getAppliedOutput()*m_turnMotor.getBusVoltage() 
         + m_driveMotor.getAppliedOutput()*m_driveMotor.getBusVoltage();
  }
}
