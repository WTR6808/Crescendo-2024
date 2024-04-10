// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Sensors.FieldManagementSystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoSearchForAmp extends Command {
  /** Creates a new SearchForTarget. */
  private SwerveDriveSubsystem m_swerve;

  //Get Instance of Field Management System Information
  FieldManagementSystem FMSInfo = FieldManagementSystem.getInstance();

  private int m_desiredTagId = 0;
  
  public AutoSearchForAmp(SwerveDriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.stop();
    //Set the Heading to 0 so we can stop after one full rotation
    m_swerve.reset_pigeon2();
    m_swerve.InitDriveToAmp();//m_swerve.setPipeline(Constants.Limelight_Constants.AMP_PIPELINE);
    if (FMSInfo.isRedAlliance()){
      m_swerve.drive(0, 0, 0.5, false);
      m_desiredTagId = 5;
    } else {
      m_swerve.drive(0, 0, -0.5, false);
      m_desiredTagId = 6;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stopDriveToTarget();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_swerve.targetAcquired());// && m_swerve.targetID() == m_desiredTagId));// || Math.abs(m_swerve.getPigeonHeading()) > 360.0);
  }
}
