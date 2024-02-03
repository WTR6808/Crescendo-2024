// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IT_IS_A_LAUNCHER;

public class LaunchSpeaker extends Command {

  IT_IS_A_LAUNCHER m_launcher;
  private int m_timer;

  /** Creates a new LaunchSpeaker. */
  public LaunchSpeaker(IT_IS_A_LAUNCHER launcher) {
    m_launcher = launcher;
    addRequirements(m_launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_launcher.launchSpeaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_launcher.getVelocity()-Constants.launcherConstants.SPEAKER_VELOCITY) <= Constants.launcherConstants.VELOCITY_TOLERANCE && m_timer<=0){
      m_launcher.flipperUp();
      m_timer = 1;
    }
    else if(m_timer>0){
      m_timer++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer > Constants.launcherConstants.FLIP_VWOOP_TIME){
      m_launcher.flipperDown();
      m_launcher.stopLauncher();
      return true;
    } 
    else{
      return false;
    }
  }
}
