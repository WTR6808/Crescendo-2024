// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IT_IS_A_LAUNCHER extends SubsystemBase {

  private static IT_IS_A_LAUNCHER instance = null;
  /** Creates a new IT_S_A_LAUNCHER. */
  private CANSparkMax m_leftLaunchMotor = new CANSparkMax(Constants.launcherConstants.LEFT_LAUNCH_CANID, MotorType.kBrushless);
  private CANSparkMax m_rightLaunchMotor = new CANSparkMax(Constants.launcherConstants.RIGHT_LAUNCH_CANID, MotorType.kBrushless);

  private RelativeEncoder m_leftLaunchEncoder = m_leftLaunchMotor.getEncoder();
  private RelativeEncoder m_rightLaunchEncoder = m_rightLaunchMotor.getEncoder();

  private SparkPIDController m_leftPidController = m_leftLaunchMotor.getPIDController();
  private SparkPIDController m_rightPidController = m_rightLaunchMotor.getPIDController();
  
  public static IT_IS_A_LAUNCHER Instance() {
    if (instance == null) {
      instance = new IT_IS_A_LAUNCHER();
    }
    return instance;
  }

  private IT_IS_A_LAUNCHER() {
    m_leftLaunchMotor.restoreFactoryDefaults();
    m_rightLaunchMotor.restoreFactoryDefaults();
    m_leftLaunchMotor.setIdleMode(IdleMode.kCoast);
    m_rightLaunchMotor.setIdleMode(IdleMode.kCoast);
    m_leftLaunchMotor.setInverted(Constants.launcherConstants.LEFT_LAUNCH_INVERTED);
    m_rightLaunchMotor.setInverted(Constants.launcherConstants.RIGHT_LAUNCH_INVERTED);
    m_leftLaunchMotor.setClosedLoopRampRate(0.08);
    m_rightLaunchMotor.setClosedLoopRampRate(0.08);
    m_leftLaunchMotor.setSmartCurrentLimit(60);
    m_rightLaunchMotor.setSmartCurrentLimit(60);
    m_leftLaunchEncoder.setVelocityConversionFactor(Constants.launcherConstants.LAUNCHER_REDUCTION);
    m_rightLaunchEncoder.setVelocityConversionFactor(Constants.launcherConstants.LAUNCHER_REDUCTION);

    m_rightPidController.setP(0);
    m_rightPidController.setI(0);
    m_rightPidController.setD(0);
    m_rightPidController.setIZone(0);
    m_rightPidController.setFF(0);
    m_rightPidController.setOutputRange(-1,1);

    m_leftPidController.setP(0);
    m_leftPidController.setI(0);
    m_leftPidController.setD(0);
    m_leftPidController.setIZone(0);
    m_leftPidController.setFF(0);
    m_leftPidController.setOutputRange(-1,1);

    stopLauncher();
 }

  public void resetEncoders(){
    m_leftLaunchEncoder.setPosition(0);
    m_rightLaunchEncoder.setPosition(0);
  }

  public void launchSpeaker(){
    m_rightPidController.setP(Constants.launcherConstants.SPEAKER_LAUNCH_P);
    m_rightPidController.setI(Constants.launcherConstants.SPEAKER_LAUNCH_I);
    m_rightPidController.setD(Constants.launcherConstants.SPEAKER_LAUNCH_D);
    m_rightPidController.setIZone(Constants.launcherConstants.SPEAKER_LAUNCH_IZONE);
    m_rightPidController.setFF(Constants.launcherConstants.SPEAKER_FEEDFORWARD);
    m_rightPidController.setReference(Constants.launcherConstants.SPEAKER_VELOCITY,CANSparkMax.ControlType.kVelocity);

    m_leftPidController.setP(Constants.launcherConstants.SPEAKER_LAUNCH_P);
    m_leftPidController.setI(Constants.launcherConstants.SPEAKER_LAUNCH_I);
    m_leftPidController.setD(Constants.launcherConstants.SPEAKER_LAUNCH_D);
    m_leftPidController.setIZone(Constants.launcherConstants.SPEAKER_LAUNCH_IZONE);
    m_leftPidController.setFF(Constants.launcherConstants.SPEAKER_FEEDFORWARD); 
    m_leftPidController.setReference(Constants.launcherConstants.SPEAKER_VELOCITY, CANSparkMax.ControlType.kVelocity);
  }

  public void launchAmp(){
    m_rightPidController.setP(Constants.launcherConstants.AMP_LAUNCH_P);
    m_rightPidController.setI(Constants.launcherConstants.AMP_LAUNCH_I);
    m_rightPidController.setD(Constants.launcherConstants.AMP_LAUNCH_D);
    m_rightPidController.setIZone(Constants.launcherConstants.AMP_LAUNCH_IZONE);
    m_rightPidController.setFF(Constants.launcherConstants.AMP_FEEDFORWARD);
    m_rightPidController.setReference(Constants.launcherConstants.AMP_VELOCITY, CANSparkMax.ControlType.kVelocity);

    m_leftPidController.setP(Constants.launcherConstants.AMP_LAUNCH_P);
    m_leftPidController.setI(Constants.launcherConstants.AMP_LAUNCH_I);
    m_leftPidController.setD(Constants.launcherConstants.AMP_LAUNCH_D);
    m_leftPidController.setIZone(Constants.launcherConstants.AMP_LAUNCH_IZONE);
    m_leftPidController.setFF(Constants.launcherConstants.AMP_FEEDFORWARD);
    m_leftPidController.setReference(Constants.launcherConstants.AMP_VELOCITY, CANSparkMax.ControlType.kVelocity);
  }

  public void reverseLauncher(){
    stopLauncher(); 
    m_leftLaunchMotor.set(-.85);
    m_rightLaunchMotor.set(-.85);
  }

  public void stopLauncher(){
    m_leftLaunchMotor.set(0);
    m_rightLaunchMotor.set(0);
    m_rightPidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_leftPidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Velocity", m_leftLaunchEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightLaunchEncoder.getVelocity());
  }
}
