// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoResetPigeon2 extends InstantCommand {
  private SwerveDriveSubsystem m_swerve;
  private NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  private NetworkTableEntry IsRedAlliance  = FMSInfo.getEntry("IsRedAlliance");
  //Creates a new AutoResetPigeon2
  public AutoResetPigeon2(SwerveDriveSubsystem swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //If we are the red alliance, initialize the angle to -90, +90 for blue
    //However, since the Pigeon2 is inverted from robot coordinates, these need
    //to be red +90, blue -90.
    if (IsRedAlliance.getBoolean(false)){
      m_swerve.setPigeon2Angle(90.0);
    } else {
      m_swerve.setPigeon2Angle(-90.0);
    } 
  }
}
