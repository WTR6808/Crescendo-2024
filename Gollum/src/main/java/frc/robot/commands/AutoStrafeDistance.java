// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoStrafeDistance extends Command {
  private SwerveDriveSubsystem m_swerve;
  private double m_distance = 0;
  private PIDController distancePID = new PIDController(1,0,0);

  private NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  private NetworkTableEntry IsRedAlliance  = FMSInfo.getEntry("IsRedAlliance");

  /** Creates a new StrafeDistance. */
  public AutoStrafeDistance(SwerveDriveSubsystem swerve, double dist) {
    m_distance = dist;
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.stop();
    m_swerve.resetEncoders();
    //If we are the blue alliance, move in negative direction
    if (!IsRedAlliance.getBoolean(false)){
      m_distance *= -1;
    }
    distancePID.setSetpoint(Units.inchesToMeters(m_distance));
    distancePID.setTolerance(Units.inchesToMeters(1.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s = MathUtil.clamp(distancePID.calculate(m_swerve.getAvgDistance()),-0.7,0.7);
    if (!distancePID.atSetpoint()){
      if (Math.abs(s)<0.4){
        s=Math.signum(s)*0.4;
      }
    }
    m_swerve.drive(0.0, s, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (distancePID.atSetpoint());
  }
}
