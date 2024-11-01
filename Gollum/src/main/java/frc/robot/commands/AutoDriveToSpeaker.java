// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveToSpeaker extends Command {
  SwerveDriveSubsystem m_swerve = null;
  boolean m_atTarget = false;
  /** Creates a new DriveToTarget. */
  public AutoDriveToSpeaker(SwerveDriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements((m_swerve));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_atTarget = false;
//    m_swerve.initAutoDriveToTarget();
    m_swerve.InitAutoDriveToSpeaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_atTarget = !m_swerve.driveToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stopDriveToTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_atTarget;
  }
}
