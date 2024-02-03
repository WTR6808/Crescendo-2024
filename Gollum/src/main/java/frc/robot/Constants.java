// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DEADZONE = 0.3;
  }

  public static class Measurements {
    public static final double TRACK_WIDTH    = Units.inchesToMeters(22.50);
    public static final double WHEEL_BASE     = Units.inchesToMeters(22.25);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    public static final double ROBOT_MAX_LINEAR_VELOCITY  = 5620.0/60.0 *
                                                            SDSModuleConstants.MK4_L3_DRIVE_REDUCTION *
                                                            WHEEL_DIAMETER * Math.PI;

    public static final double ROBOT_MAX_ANGULAR_VELOCITY = ROBOT_MAX_LINEAR_VELOCITY / Math.hypot(TRACK_WIDTH/2.0, WHEEL_BASE/2.0);

    public static final double ROBOT_MAX_LAUNCH_VELOCITY = 5620.0/60.0 *
                                                          launcherConstants.LAUNCHER_REDUCTION *
                                                          WHEEL_DIAMETER * Math.PI; //TODO, not sure if this is right
    
  }

  public static class SwerveDriveConstants {
    //CANID for Pigeon IMU
    public static final int PIGEON_CANID = 0;

    //CANID's for Swerve Motors & CANCoders
    public static final int FRONT_RIGHT_DRIVE_CANID        = 11;
    public static final int FRONT_RIGHT_TURN_CANID         = 12;
    public static final int FRONT_RIGHT_ENCODER_CAN_ID     = 13;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    public static final boolean FRONT_RIGHT_TURN_INVERTED  = false;
    public static final double FRONT_RIGHT_OFFSET           = -0.280;//-0.338; //0.489//-0.385

    public static final int BACK_RIGHT_DRIVE_CANID         = 21;
    public static final int BACK_RIGHT_TURN_CANID          = 22;
    public static final int BACK_RIGHT_ENCODER_CAN_ID      = 23;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED  = false;
    public static final boolean BACK_RIGHT_TURN_INVERTED   = false;
    public static final double BACK_RIGHT_OFFSET           = -0.439;//0.480;//-0.43;//-0.454

    public static final int BACK_LEFT_DRIVE_CANID          = 31;
    public static final int BACK_LEFT_TURN_CANID           = 32;
    public static final int BACK_LEFT_ENCODER_CAN_ID       = 33;
    public static final boolean BACK_LEFT_DRIVE_INVERTED   = true;
    public static final boolean BACK_LEFT_TURN_INVERTED    = false;
    public static final double BACK_LEFT_OFFSET           = 0.181;//0.214; //0.171//0.208

    public static final int FRONT_LEFT_DRIVE_CANID         = 41;
    public static final int FRONT_LEFT_TURN_CANID          = 42;
    public static final int FRONT_LEFT_ENCODER_CAN_ID      = 43;
    public static final boolean FRONT_LEFT_DRIVE_INVERTED  = true;
    public static final boolean FRONT_LEFT_TURN_INVERTED   = false;
    public static final double FRONT_LEFT_OFFSET           = 0.320; //0.341;//0.300; //.260//0.294

    //CANID's for intake motors
    public static final int FRONT_RIGHT_INTAKE_CANID       = 51;
    public static final int BACK_RIGHT_INTAKE_CANID        = 52;
    public static final int BACK_LEFT_INTAKE_CANID         = 53;
    public static final int FRONT_LEFT_INTAKE_CANID        = 54;
    public static final boolean FRONT_RIGHT_INTAKE_INVERTED  = false;
    public static final boolean BACK_RIGHT_INTAKE_INVERTED   = false;
    public static final boolean BACK_LEFT_INTAKE_INVERTED    = false;
    public static final boolean FRONT_LEFT_INTAKE_INVERTED   = false;

    //Robot Kinematic Constants
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d( Measurements.WHEEL_BASE/2.0,  Measurements.TRACK_WIDTH/2.0), //Front Left
            new Translation2d( Measurements.WHEEL_BASE/2.0, -Measurements.TRACK_WIDTH/2.0), //Front Right
            new Translation2d(-Measurements.WHEEL_BASE/2.0,  Measurements.TRACK_WIDTH/2.0), //Back Left
            new Translation2d(-Measurements.WHEEL_BASE/2.0, -Measurements.TRACK_WIDTH/2.0)  //Back Right
        );
  }

  public static class SDSModuleConstants {
    public static final double MK4_L3_DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double MK4_L3_TURN_REDUCTION  = (15.0 / 32.0) * (10.0 / 60.0);
    public static final boolean MK4_L3_DRIVE_INVERTED = true;
    public static final boolean MK4_L3_TURN_INVERTED  = true;

    //Encoder Constants
    public static final double DRIVE_ENCODER_CPR = 42.0; //NEO Built In
    public static final double TURN_ENCODER_CPR  = 42.0; //NEO Built In
  
    //Linear Distance Conversion Factor is Computed Using the Following Equation:
    //  Wheel Diameter * PI * Gear Ratio / Encoder's Counts per Revolutions
    //    Units are in Wheel Diameter Units
    public static final double DRIVE_DISTANCE_CONVERSION = (Measurements.WHEEL_DIAMETER * Math.PI * MK4_L3_DRIVE_REDUCTION) / DRIVE_ENCODER_CPR;
    
    //Linear Velocity Conversion Factor is Computed Using the Following Equation
    //  RPM/60 seconds * (PI * Wheel Diameter) * Gear Ratio
    //    Units are in Wheel Diameter Units/Second
    public static final double DRIVE_VELOCITY_CONVERSION = (Measurements.WHEEL_DIAMETER * Math.PI * MK4_L3_DRIVE_REDUCTION)/60.0;


    //Angular Distance Conversion Factor is Computed Using the Following Equation:
    // 2 * PI * Gear Ratio / Encoder's Counts per Revolutions
    //    Units are in Radians
    //When switching to CANCoder, Gear Ration is 1.0 since CANCoder is mounted directly to turn shaft
    public static final double TURN_DISTANCE_CONVERSION = (2 * Math.PI * MK4_L3_TURN_REDUCTION * (2*Math.PI/6.31));// / TURN_ENCODER_CPR;
    // public static final double TURN_DISTANCE_CONVERSION = (2 * Math.PI * MK4_L3_TURN_REDUCTION) / TURN_ENCODER_CPR;

    //PID Constants for Drive and Turn Motors
    //FIXME Need to tune these values
    public static final double DRIVE_P = 1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0005;

    public static final double TURN_P  = 0.6;
    public static final double TURN_I  = 0.0;
    public static final double TURN_D  = 0.0;
  }

  public class launcherConstants {

    public static final double LAUNCHER_REDUCTION = 1.0;
    public static final double LAUNCH_VELOCITY_CONVERSION = (Measurements.WHEEL_DIAMETER * Math.PI * LAUNCHER_REDUCTION)/60.0;
  
    //CANID's for launcher motors
    public static final int LEFT_LAUNCH_CANID         = 61;
    public static final int RIGHT_LAUNCH_CANID        = 62;
    public static final boolean LEFT_LAUNCH_INVERTED  = true;
    public static final boolean RIGHT_LAUNCH_INVERTED = false;

    //PID Constants for Launcher Motors
    public static final double SPEAKER_LAUNCH_P     =    0.0001;
    public static final double SPEAKER_LAUNCH_I     =    0.0;
    public static final double SPEAKER_LAUNCH_D     =    0.0;
    public static final double SPEAKER_LAUNCH_IZONE =    0.0;
    public static final double SPEAKER_FEEDFORWARD  =    0.00017; //0.00125/4.0;
    public static final double SPEAKER_VELOCITY     = 4800.0; //5500;

    public static final double AMP_LAUNCH_P         =    0.0001;
    public static final double AMP_LAUNCH_I         =    0.0;
    public static final double AMP_LAUNCH_D         =    0.0;
    public static final double AMP_LAUNCH_IZONE     =    0.0;
    public static final double AMP_FEEDFORWARD      =    0.00017;
    public static final double AMP_VELOCITY         = 1000.0;
    //TODO find velocity constants for launcher
  }
}