// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IT_IS_A_LAUNCHER;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final Joystick m_Joystick = 
      new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

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
       //FIXME Use controller values for all axis temp only use one at a time to debug
        new RunCommand(
            () ->
                m_swerve.drive(
                    /*
                    m_Joystick.getX(),
                    m_Joystick.getY(), 
                    m_Joystick.getZ(),
                     */
                    -m_driverController.getLeftY(), 
                    -m_driverController.getLeftX(),
                    -m_driverController.getRightX(),
                    true),
                m_swerve));
    SmartDashboard.putData("Reset Encoders", new InstantCommand(()->m_swerve.resetEncoders(), m_swerve));
    SmartDashboard.putData("Reset Gyro (Pigeon2)", new InstantCommand(()->m_swerve.reset_pigeon2(), m_swerve));
    SmartDashboard.putData("Run Launcher Speaker", new InstantCommand(()->m_launcher.launchSpeaker(), m_launcher));
    SmartDashboard.putData("Run Launcher Amp", new InstantCommand(()->m_launcher.launchAmp(), m_launcher));
    SmartDashboard.putData("Stop Launcher", new InstantCommand(()->m_launcher.stopLauncher(), m_launcher));
    SmartDashboard.putData("Run Launcher Reverse", new InstantCommand(()->m_launcher.reverseLauncher(), m_launcher));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
