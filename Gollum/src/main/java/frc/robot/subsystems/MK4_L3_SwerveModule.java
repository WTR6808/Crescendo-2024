// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private final PIDController m_drivePID = new PIDController(Constants.SDSModuleConstants.DRIVE_P,
                                                             Constants.SDSModuleConstants.DRIVE_I,
                                                             Constants.SDSModuleConstants.DRIVE_D);
  
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
    //FIXME m_driveMotor.setSmartCurrentLimit(xx);
    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(Constants.SDSModuleConstants.DRIVE_DISTANCE_CONVERSION);
    m_turnEncoder.setVelocityConversionFactor(Constants.SDSModuleConstants.DRIVE_VELOCITY_CONVERSION);
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetEncoders(){
    m_driveEncoder.setPosition(0.0);
    m_turnEncoder.setPosition(0.0);
  }

  public void writeToSmartDashboard(){}

  //TODO Replace this routine for Actual Swerve
  public void setDesiredState(SwerveModuleState desiredState){
    double driveOutput, turnOutput;
    var encoderRotation = new Rotation2d(m_turnEncoder.getPosition());
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,encoderRotation);
    driveOutput = m_drivePID.calculate(m_driveEncoder.getVelocity(),state.speedMetersPerSecond);
    driveOutput = driveOutput / Constants.Measurements.ROBOT_MAX_LINEAR_VELOCITY;

    turnOutput = m_drivePID.calculate(m_turnEncoder.getPosition(),state.angle.getRadians());
    turnOutput = turnOutput / Math.abs(state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turnMotor.set(turnOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
