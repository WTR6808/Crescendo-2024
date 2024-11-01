// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candy_Cane;

public class ClimberDown extends Command {
  /** Creates a new ClimberDown. */
  private Candy_Cane m_CC;

  public ClimberDown(Candy_Cane c) {
    m_CC = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_CC);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CC.climberDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CC.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_CC.getLeftEncoderPos()>110;
  }
}
