// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  //FIXME Change to Absolute CTRE Encoder for final code
  private RelativeEncoder m_turnEncoder;

  //FIXME Change to on SparkMax PID Controller?
  private final PIDController m_drivePID = new PIDController(Constants.SDSModuleConstants.DRIVE_P,
                                                             Constants.SDSModuleConstants.DRIVE_I,
                                                             Constants.SDSModuleConstants.DRIVE_D);
  
  //FIXME Change to Profiled PID Controller after initial tuning?
  private final PIDController m_turnPID = new PIDController(Constants.SDSModuleConstants.TURN_P,
                                                            Constants.SDSModuleConstants.TURN_I,
                                                            Constants.SDSModuleConstants.TURN_D);

  /** Creates a new MK4_L3_SwerveModule. */
  public MK4_L3_SwerveModule(String name,
                             int driveCanID,
                             boolean driveInverted,
                             int turnCanID,
                             boolean turnInverted,
                             int turnEncoderID) {
    m_name = name;
    m_driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(driveInverted);
    //FIXME m_driveMotor.setSmartCurrentLimit(xx);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Constants.SDSModuleConstants.DRIVE_DISTANCE_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(Constants.SDSModuleConstants.DRIVE_VELOCITY_CONVERSION);

    m_turnMotor = new CANSparkMax(turnCanID, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setIdleMode(IdleMode.kBrake);
    m_turnMotor.setInverted(turnInverted);
    //FIXME m_turnMotor.setSmartCurrentLimit(xx);
    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(Constants.SDSModuleConstants.TURN_DISTANCE_CONVERSION);
    //m_turnEncoder.setVelocityConversionFactor(Constants.SDSModuleConstants.DRIVE_VELOCITY_CONVERSION);
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);

    //TODO Point out that we forgot this on 1/13/2024
    resetEncoders();
  }

  public void resetEncoders(){
    m_driveEncoder.setPosition(0.0);
    m_turnEncoder.setPosition(0.0);
  }

  //TODO Replace this routine for Actual Swerve
  public void setDesiredState(SwerveModuleState desiredState){
    double driveOutput, turnOutput;
    Rotation2d encoderRotation = new Rotation2d(m_turnEncoder.getPosition());
    double driveVelocity = m_driveEncoder.getVelocity();

    SwerveModuleState state = SwerveModuleState.optimize(desiredState,encoderRotation);
    //FIXME For debug/testing remove PID
    driveOutput = m_drivePID.calculate(driveVelocity,state.speedMetersPerSecond);
    //driveOutput = state.speedMetersPerSecond;
    driveOutput = driveOutput / Constants.Measurements.ROBOT_MAX_LINEAR_VELOCITY;

    turnOutput = MathUtil.clamp(m_turnPID.calculate(encoderRotation.getRadians(),state.angle.getRadians()),-1.0,1.0);
    //turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
    //turnOutput = (state.angle.getRadians()==0)?0.0:turnOutput / Math.abs(state.angle.getRadians());

    //Send States to SmartDashboard
    SmartDashboard.putNumber(m_name+" Desired Speed: ", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name+" Desired Angle: ", desiredState.angle.getDegrees());
    SmartDashboard.putNumber(m_name+" Optimized Speed:", state.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name+" Optimized Angle: ", state.angle.getDegrees());

    SmartDashboard.putNumber(m_name+" Drive Encoder Vel: ", driveVelocity);
    SmartDashboard.putNumber(m_name+" Turn Encoder Pos: ", encoderRotation.getRadians());
    SmartDashboard.putNumber(m_name+" Drive Output: ", driveOutput);
    SmartDashboard.putNumber(m_name+" Turn Output: ", turnOutput);

    m_driveMotor.set(driveOutput);
    m_turnMotor.set(turnOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
