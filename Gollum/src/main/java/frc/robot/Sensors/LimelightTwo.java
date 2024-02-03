// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Sensors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;



/** Add your docs here. */
public class LimelightTwo {
    //Create instance for Singleton
    private static LimelightTwo instance = null;

    private static final double TX_MAX       = 29.80;
    private static final double TY_MAX       = 24.85;
    private static final double X_TOLERANCE  =  1.0;
    private static final double Y_TOLERANCE  =  1.0;
    private static final int    MAX_FAILURES = 20;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");

    private static final double MAX_DRIVE         =  0.75; //0.75 PracticeBot
	private static final double MIN_DRIVE         =  0.40; //0.40 PracticeBot
	private static final double MAX_STEER         =  0.60;
	private static final double MIN_STEER         =  0.45;//0.30; //0.30 PracticeBot
	//private static final double MIN_STEER_STOPPED =  0.45; //0.45 PracticeBot

    private double m_driveCommand  = 0.0;
    private double m_steerCommand  = 0.0;
    private double m_strafeCommand = 0.0;

    private int failureCount = 0;

	private PIDController m_drivePID = new PIDController(1.0, 0.0, 0.0);
	private PIDController m_steerPID = new PIDController(1.0, 0.0, 0.0);

    /** Creates a new SwerveDriveSubsystem. */
    public static LimelightTwo Instance() {
        if (instance == null){ 
            instance = new LimelightTwo();
        }
        return instance;
    }

    //Make Constructor Private to enforce Singleton behavior
    private LimelightTwo(){
		//Set the tolerance & Set Points for the PID Controllers
		m_drivePID.setTolerance(Y_TOLERANCE);
		m_drivePID.setSetpoint(0.0);
		m_steerPID.setTolerance(X_TOLERANCE);
		m_steerPID.setSetpoint(0.0);

        //Set LEDs to use Pipeline Setting
        table.getEntry("ledMode").setNumber(0);

        //Initialize Target Tracking to Off
//TODO Uncomment when Constants Updated
//        setTrackTarget(false, Constants.LimelightConstants.DRIVE_PIPELINE);
    }

	public double steerCommand(){
        //Left/Right Steering Speed
        return m_steerCommand;
    }

    public double driveCommand(){
        //Forward/Reverse Driving Speed
        return m_driveCommand;
    }

    public double strafeCommand(){
        return m_strafeCommand;
    }
  
    public double steerError(){
        //X axis error from crosshair
        //  Used to compute Steering Speed
        return tx.getDouble(0.0);
    }
                                    
    public double distanceError(){
        //Y axis error from crosshair
        //  Used to compute Drive Speed
        return ty.getDouble(0.0);
    }
  
    public boolean targetAcquired(){
        //tv = 1.0, then target aquired
        return (tv.getDouble(0.0) > 0.0);
    }
  
    public void setLEDs(boolean on){
        //Value of 3 turns LEDs on, 1 turns LEDs off
        if (on){
            table.getEntry("ledMode").setNumber(3);
        }else{
            table.getEntry("ledMode").setNumber(1);
        }
    }

    public void setTrackTarget(boolean on, int pipeline){
        //setnumber filled with pipeline #
        table.getEntry("pipeline").setNumber(pipeline);
		//Reset the PID controllers
		m_drivePID.reset();
		m_steerPID.reset();
        //Reset Failure Count
        failureCount = 0;
    }
	
    public boolean atTarget(){

        return targetAcquired() && m_drivePID.atSetpoint() && m_steerPID.atSetpoint();
    }
	public boolean calcMotorControl(){
        //Perform calculations if there is a target and at least one of the errors is not within tolerance
        boolean validResult  = targetAcquired() && (!m_drivePID.atSetpoint() || !m_steerPID.atSetpoint());
		double driveCommand  = 0.0;
		double steerCommand  = 0.0;
        double strafeCommand = 0.0;

        if (validResult){
            failureCount = 0;
			//Calculate the next PID measurement for Driving Forward
			if (!m_drivePID.atSetpoint()){
 				driveCommand = m_drivePID.calculate(distanceError()) / TY_MAX;
            	if (Math.abs(driveCommand) < MIN_DRIVE){
					driveCommand = Math.signum(driveCommand)*MIN_DRIVE; 
				} else {
                    driveCommand = MathUtil.clamp(driveCommand, -MAX_DRIVE, MAX_DRIVE);
                }
			}
			
			//Calculate the next PID measurement for Turning
			if (!m_steerPID.atSetpoint()){
               steerCommand = m_steerPID.calculate(steerError()) / TX_MAX;
                if (Math.abs(steerCommand) < MIN_STEER) {
					steerCommand = Math.signum(steerCommand)*MIN_STEER;
                } else {
                    steerCommand = MathUtil.clamp(steerCommand, -MAX_STEER, MAX_STEER);
				}

                //If at correct distance, then use rotation and strafing
                if (m_drivePID.atSetpoint()){
                    //TODO need to verify if this is correct and if speed should be adjusted
                    strafeCommand = steerCommand;
                    steerCommand = -steerCommand;
                } 
			}
		} else {
			validResult = (++failureCount <= MAX_FAILURES);
		}
        
        m_driveCommand  = driveCommand;
        m_steerCommand  = steerCommand;
        m_strafeCommand = strafeCommand;

		return validResult;
	}
}

