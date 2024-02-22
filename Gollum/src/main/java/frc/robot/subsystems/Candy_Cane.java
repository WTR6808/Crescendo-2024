// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candy_Cane extends SubsystemBase {


  private static Candy_Cane instance = null;
  /** Creates a new Candy_Cane. */
  private CANSparkMax m_leftClimber = new CANSparkMax(Constants.Climber.LEFT_CLIMBER_CANID, MotorType.kBrushless);
  private CANSparkMax m_rightClimber = new CANSparkMax(Constants.Climber.RIGHT_CLIMBER_CANID, MotorType.kBrushless);

  private RelativeEncoder m_leftClimberEncoder = m_leftClimber.getEncoder();
  private RelativeEncoder m_rightClimberEncoder = m_rightClimber.getEncoder();

  public static Candy_Cane Instance() {
    if (instance == null) {
      instance = new Candy_Cane();
    }
    return instance;
  }

  private Candy_Cane(){
    m_leftClimber.restoreFactoryDefaults();
    m_rightClimber.restoreFactoryDefaults();
    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);
    m_leftClimber.setInverted(Constants.Climber.LEFT_CLIMBER_INVERTED);
    m_rightClimber.setInverted(Constants.Climber.RIGHT_CLIMBER_INVERTED);
    m_leftClimber.setSmartCurrentLimit(60);
    m_rightClimber.setSmartCurrentLimit(60);
    m_rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 200);
    m_rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    m_leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 180);
    m_leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    m_leftClimber.follow(m_rightClimber);
  }

  public void resetEncoders(){
    m_leftClimberEncoder.setPosition(0);
    m_rightClimberEncoder.setPosition(0);
  }

public void climberUp(){
    m_rightClimber.set(-0.5);
}

public void climberDown(){
  m_rightClimber.set(0.5);
}

public void stopClimber(){
  m_leftClimber.set(0);
  m_rightClimber.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
