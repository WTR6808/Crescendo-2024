// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto1;
import frc.robot.commands.AutoDriveToSpeaker;
import frc.robot.commands.AutoSearchForAmp;
import frc.robot.commands.AutoStrafeDistance;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.subsystems.Candy_Cane;
import frc.robot.subsystems.IT_IS_A_LAUNCHER;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveDriveSubsystem m_swerve = SwerveDriveSubsystem.Instance();
  IT_IS_A_LAUNCHER m_launcher = IT_IS_A_LAUNCHER.Instance();
  Candy_Cane m_Candy_Cane = Candy_Cane.Instance();
  LEDSubsystem m_leds = LEDSubsystem.Instance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final Joystick m_Joystick = 
      new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Configure default commands
    m_swerve.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

       // Multiply by max speed to map the joystick unitless inputs to actual units.
       // This will map the [-1, 1] to [max speed backwards, max speed forwards],
       // converting them to actual units.
        new RunCommand(
            () ->
                m_swerve.drive(
                    /* */
                    -m_Joystick.getY(),
                    -m_Joystick.getX(), 
                    -m_Joystick.getZ(),
                     /* */ /* 
                    -m_driverController.getLeftY(), 
                    -m_driverController.getLeftX(),
                    -m_driverController.getRightX(),
                    */
                    true),
                m_swerve));

    SmartDashboard.putData("Reset Encoders", new InstantCommand(()->m_swerve.resetEncoders(), m_swerve));
    SmartDashboard.putData("Reset Gyro (Pigeon2)", new InstantCommand(()->m_swerve.reset_pigeon2(), m_swerve));
    SmartDashboard.putData("Run Launcher Speaker", new LaunchSpeaker(m_launcher));
    SmartDashboard.putData("Run Launcher Amp", new LaunchAmp(m_launcher));
    SmartDashboard.putData("Stop Launcher", new InstantCommand(()->m_launcher.stopLauncher(), m_launcher));
    SmartDashboard.putData("Run Launcher Reverse", new InstantCommand(()->m_launcher.reverseLauncher(), m_launcher));
    SmartDashboard.putData("Test Drive to Target",new DriveToTarget(m_swerve));
    SmartDashboard.putData("Stop Climber", new InstantCommand(()->m_Candy_Cane.stopClimber(),m_Candy_Cane));
    SmartDashboard.putData("Take Snap Shot", new InstantCommand(()->m_swerve.takeSnapShot(),m_swerve));
    SmartDashboard.putData("Test Drive to Auto Target",new AutoDriveToSpeaker(m_swerve));
    
    SmartDashboard.putData("Test Auto Drive to Speaker",new AutoDriveToSpeaker(m_swerve));
    SmartDashboard.putData("Test Drive Distance (1.5 feet)",new DriveDistance(m_swerve, 18));
    SmartDashboard.putData("Test Strafe Distance (2 feet)", new AutoStrafeDistance(m_swerve, 24));
    SmartDashboard.putData("Test Search for Amp", new AutoSearchForAmp(m_swerve));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //*****************Driver Joystick Button Assignments**********************
    //DriverResetGyro Zeroes the Gyro as is intended for the Driver to calibrate Field Centric Operation 
    final JoystickButton DriverResetGyro = new JoystickButton(m_Joystick, Constants.Buttons.BUTTON_RESET_GYRO);
    DriverResetGyro.onTrue(new InstantCommand(()->m_swerve.reset_pigeon2(), m_swerve));
    
    //DriveToApril aligns the robot to the desired launching position based on the currently detected April Tag
    final JoystickButton DriveToApril = new JoystickButton(m_Joystick, 1);
    DriveToApril.onTrue(new DriveToTarget(m_swerve).withTimeout(2.0));

    //*****************Operator Controller Button Assignments**********************
    //Make sure you're pulling a public routine, always double check, I'm looking at you Molly, and I'm judging you : )
    //Left and Right Bumpers launch a note into the Amp or Speaker respectively
    m_operatorController.leftBumper().onTrue(new LaunchAmp(m_launcher));
    m_operatorController.rightBumper().onTrue(new LaunchSpeaker(m_launcher));
    //m_operatorController.leftBumper().whileFalse(new InstantCommand(()->m_launcher.stopLauncher(), m_launcher));

    //B when pressed runs the launcher in reverse to receive a Note from the Source and stops when released
    m_operatorController.b().whileTrue(new InstantCommand(()->m_launcher.reverseLauncher(),m_launcher))
                            .onFalse(new InstantCommand(()-> m_launcher.stopLauncher(),m_launcher));

    //POV Up while true will raise the climber arms (lower robot) and stop when released
    //OPERATOR IS RESPONSIBLE FOR STOPPING BY RELEASING THE POV BUTTON
    m_operatorController.y().whileTrue(new InstantCommand(()->m_Candy_Cane.climberUp(), m_launcher))
                            .onFalse(new InstantCommand(()->m_Candy_Cane.stopClimber(), m_launcher));
    //POV Down while true will lower the climber arms (lift robot) and stop when released
    //OPERATOR IS RESPONSIBLE FOR STOPPING BY RELEASING THE POV BUTTON
    m_operatorController.a().whileTrue(new InstantCommand(()->m_Candy_Cane.climberDown(), m_launcher))
                            .onFalse(new InstantCommand(()->m_Candy_Cane.stopClimber(), m_launcher));

    //Commented POV Commands Below have safety stops in them to prevent over
    //tightening the ropes.  Had problems 2/26 with encoder positions not being
    //reliable, so replaced with above.  If we can figure out encoder issues,
    //go back to these routines.
    m_operatorController.povUp().onTrue(new ClimberUp(m_Candy_Cane));
    m_operatorController.povDown().onTrue(new ClimberDown(m_Candy_Cane));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return new Auto1(m_swerve, m_launcher);
    //return null;
  }
}
