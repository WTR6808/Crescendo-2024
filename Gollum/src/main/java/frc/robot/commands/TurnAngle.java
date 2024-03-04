// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TurnAngle extends Command {
  private PIDController anglePID = new PIDController(1, 0, 0);
  private SwerveDriveSubsystem m_swerve;
  private double m_angle;
  private NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  private NetworkTableEntry IsRedAlliance  = FMSInfo.getEntry("IsRedAlliance");
  /** Creates a new TurnAngle. */
  public TurnAngle(SwerveDriveSubsystem s) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = s;
    //m_angle = a;
    addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.setSetpoint(m_angle);
    anglePID.setTolerance(2);
    m_swerve.reset_pigeon2();
    if (IsRedAlliance.getBoolean(false)){//red is positive
      anglePID.setSetpoint(45.0);
      
    } else {
      anglePID.setSetpoint(-45.0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(anglePID.calculate(m_swerve.getPigeonHeading()), -0.7, 0.7);
    if(!anglePID.atSetpoint()){
      if(Math.abs(speed)<0.5){
        speed = Math.signum(speed)*0.5;
      }
      m_swerve.drive(0.0, 0.0, speed, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint();
  }
}
