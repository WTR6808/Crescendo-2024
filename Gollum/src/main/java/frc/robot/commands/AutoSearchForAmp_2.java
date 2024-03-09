// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FieldManagementSystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoSearchForAmp_2 extends Command {
  private SwerveDriveSubsystem m_swerve = null;
  private int m_latency;
  private double m_rotation;
  private static final int MAX_LATENCY = 10; //Approximately 10 cycles or 500ms

  //Get instance of Field Management System Information
  FieldManagementSystem FMSInfo = FieldManagementSystem.getInstance();

  /** Creates a new AutoSearchForAmp_2. */
  public AutoSearchForAmp_2(SwerveDriveSubsystem s) {
    m_swerve = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_latency = 0;
    m_swerve.InitDriveToAmp();
    //Set rotation CCW(+) if red, CW(-) if blue
    m_rotation = FMSInfo.isRedAlliance() ? 0.65: -0.65;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_latency < MAX_LATENCY){
      //Wait for next cycle
      m_latency++;
    } else {
      m_swerve.drive(0.0, 0.0, m_rotation, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stopDriveToTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Wait for Net Tables latency before checking for target
    //End when latency has passed and target is acquired.  
    //Note: If target is never found command will never end
    //      To avoid this, use withTimeout(xx);
    return (m_latency >= MAX_LATENCY) ? m_swerve.targetAcquired() : false;
  }
}
