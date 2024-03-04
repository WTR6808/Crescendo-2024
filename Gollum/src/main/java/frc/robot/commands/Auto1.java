// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IT_IS_A_LAUNCHER;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto1(SwerveDriveSubsystem Swerve, IT_IS_A_LAUNCHER Launcher) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveToSpeaker(Swerve), 
      new LaunchSpeaker(Launcher),
      new WaitCommand(1.3), 
      new TurnAngle(Swerve),
      new AutoSearchForAmp(Swerve),
      //new DriveToTarget(Swerve),
      new DriveDistance(Swerve, 36.0).withTimeout(2.0), 
      new AutoResetPigeon2(Swerve), 
      new AutoStrafeDistance(Swerve, 48.0)
    );
  }
}
