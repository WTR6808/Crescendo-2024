// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax m_rightFrontIntakeMotor;
  private CANSparkMax m_rightBackIntakeMotor;
  private CANSparkMax m_leftBackIntakeMotor;
  private CANSparkMax m_leftFrontIntakeMotor;
  private RelativeEncoder m_rightFrontIntakeEncoder; 
  private RelativeEncoder m_rightBackIntakeEncoder;
  private RelativeEncoder m_leftBackIntakeEncoder;
  private RelativeEncoder m_leftFrontIntakeEncoder;

  
  public Intake() {
m_rightFrontIntakeMotor.restoreFactoryDefaults();
m_rightBackIntakeMotor.restoreFactoryDefaults();
m_leftBackIntakeMotor.restoreFactoryDefaults();
m_leftFrontIntakeMotor.restoreFactoryDefaults();
m_rightFrontIntakeMotor.setIdleMode(IdleMode.kBrake);
m_rightBackIntakeMotor.setIdleMode(IdleMode.kBrake);
m_leftBackIntakeMotor.setIdleMode(IdleMode.kBrake);
m_leftFrontIntakeMotor.setIdleMode(IdleMode.kBrake);
m_rightFrontIntakeMotor.setInverted(Constants.SwerveDriveConstants.FRONT_RIGHT_INTAKE_INVERTED);
m_rightBackIntakeMotor.setInverted(Constants.SwerveDriveConstants.BACK_RIGHT_INTAKE_INVERTED);
m_leftBackIntakeMotor.setInverted(Constants.SwerveDriveConstants.BACK_LEFT_INTAKE_INVERTED);
m_leftFrontIntakeMotor.setInverted(Constants.SwerveDriveConstants.FRONT_LEFT_INTAKE_INVERTED);


m_rightFrontIntakeEncoder = m_rightFrontIntakeMotor.getEncoder();
m_rightBackIntakeEncoder = m_rightBackIntakeMotor.getEncoder();
m_leftBackIntakeEncoder = m_leftBackIntakeMotor.getEncoder();
m_leftFrontIntakeEncoder = m_leftFrontIntakeMotor.getEncoder();
//TODO
//m_leftLaunchEncoder.setVelocityConversionFactor(Constants.launcherConstants.LAUNCHER_REDUCTION);
//m_rightLaunchEncoder.setVelocityConversionFactor(Constants.launcherConstants.LAUNCHER_REDUCTION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
