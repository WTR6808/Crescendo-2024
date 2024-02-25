// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoSearchForAmp extends Command {
  /** Creates a new SearchForTarget. */
  private SwerveDriveSubsystem m_swerve;
  private NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  private NetworkTableEntry IsRedAlliance  = FMSInfo.getEntry("IsRedAlliance");

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
    m_swerve.setPipeline(Constants.Limelight_Constants.AMP_PIPELINE);
    if (IsRedAlliance.getBoolean(false)){
      m_swerve.drive(0, 0, 0.3, false);
    } else {
      m_swerve.drive(0, 0, -0.3, false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_swerve.targetAcquired() || Math.abs(m_swerve.getPigeonHeading()) >= 360.0);
  }
}
