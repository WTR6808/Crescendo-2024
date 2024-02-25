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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class LimelightTwo {
    //Create instance for Singleton
    private static LimelightTwo instance = null;

    private static final int TAGS_TO_PIPELINE[] = {Constants.Limelight_Constants.DEFAULT_PIPELINE,   //# 0 No target use default pipeline
                                                   Constants.Limelight_Constants.SOURCE_PIPELINE,    //# 1 Blue Source Right 
                                                   Constants.Limelight_Constants.SOURCE_PIPELINE,    //# 2 Blue Source Left  
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //# 3 Not Used by Gollum
                                                   Constants.Limelight_Constants.SPEAKER_PIPELINE_R, //# 4 Red Speaker
                                                   Constants.Limelight_Constants.AMP_PIPELINE,       //# 5 Red Amp
                                                   Constants.Limelight_Constants.AMP_PIPELINE,       //# 6 Blue Amp
                                                   Constants.Limelight_Constants.SPEAKER_PIPELINE_B, //# 7 Blue Speaker
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //# 8 Not Used by Gollum
                                                   Constants.Limelight_Constants.SOURCE_PIPELINE,    //# 9 Red Source Right
                                                   Constants.Limelight_Constants.SOURCE_PIPELINE,    //#10 Red Source Left
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //#11 Not Used by Gollum
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //#12 Not Used by Gollum
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //#13 Not Used by Gollum
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //#14 Not Used by Gollum
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE,   //#15 Not Used by Gollum
                                                   Constants.Limelight_Constants.DEFAULT_PIPELINE};  //#16 Not Used by Gollum

    private static final double TX_MAX            = 29.80;
    private static final double TY_MAX            = 24.85;
    private static final double X_TOLERANCE       =  0.5;//1.0;
    private static final double Y_TOLERANCE       =  0.5;//1.0;
    private static final int    MAX_FAILURES      = 20;
    private static final int    MAX_TARGET_LOSSES = 20;

    NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
    NetworkTableEntry IsRedAlliance  = FMSInfo.getEntry("IsRedAlliance");

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-g");
    NetworkTableEntry tx   = table.getEntry("tx");
    NetworkTableEntry ty   = table.getEntry("ty");
    NetworkTableEntry tv   = table.getEntry("tv");
    NetworkTableEntry tid  = table.getEntry("tid");
    NetworkTableEntry pipe = table.getEntry("getpipe");
    NetworkTableEntry snap = table.getEntry("snapshot");

    private static final double MAX_DRIVE         =  0.90;
	private static final double MIN_DRIVE         =  0.40;
	private static final double MAX_STEER         =  0.60;
	private static final double MIN_STEER         =  0.40;

    private double m_driveCommand  = 0.0;
    private double m_steerCommand  = 0.0;
    private double m_strafeCommand = 0.0;

    private boolean m_trackingStarted = false;
    private int m_targetLosses = MAX_TARGET_LOSSES;
    private int m_failureCount = 0;
    private int m_pipeline = 0;

	private PIDController m_drivePID = new PIDController(1.0/TY_MAX, 0.0, 0.0);
	private PIDController m_steerPID = new PIDController(1.0/TX_MAX, 0.0, 0.0);

    //Uncomment for Debug Purposes in Simulation
    //private double distErr = TY_MAX/5.0;
    //private double steerErr = TX_MAX/5.0;

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

        //Set LEDs to Use the Pipeline
        setLEDs(true);
        //Set to the Default Pipeline
        setPipeline();
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
    
        //Comment out above line and Uncomment following 2 lines for debug with Simulation
        //steerErr -= 0.1;
        //return steerErr;
    }
                                    
    public double distanceError(){
        //Y axis error from crosshair
        //  Used to compute Drive Speed
        return ty.getDouble(0.0);
        
        //Comment out above line and Uncomment following 2 lines for debug with Simulation
        //distErr -= 0.1;
        //return distErr;
    }
  
    public boolean targetAcquired(){
        //tv = 1.0, then target aquired
        //boolean target = true;//(tv.getDouble(0.0) > 0.0);
        //if (target) m_targetLosses = 0;
        //else m_targetLosses++;
        //return target || (m_targetLosses <= MAX_TARGET_LOSSES);
        double defVal = 0.0; //Change to 1.0 for Simulation Debugging
        //if (distErr < 6.0) defVal = 0.0;
        return (tv.getDouble(defVal) > 0.0);
    }
  
    public int getTagNumber(){
        if (targetAcquired()){
            return (int) tid.getInteger(0);
        }
        else{
            return 0;
        }
    }

    public void takeSnapShot(){
        snap.setInteger(0);
        snap.setInteger(1);
    }
    
    public void setLEDs(boolean on){
        //Value of 0 LEDs us Pipeline, 1 turns LEDs off
        if (on){
            //Use Pipeline Setting
            table.getEntry("ledMode").setNumber(0);
        }else{
            //Turn Off
            table.getEntry("ledMode").setNumber(1);
        }
    }

    public int getCurrentPipeline(){
        return (int) pipe.getInteger(0);
    }

    public void setPipeline(){
        //setnumber filled with pipeline #
        //table.getEntry("pipeline").setInteger(m_pipeline);
        pipe.setInteger(m_pipeline);
    }

    public void setPipeline(int pipe){
        m_pipeline = pipe;
        setPipeline();
    }

    public void initializeTargetTracking(){
        //Get the pipeline based on the current tracked April Tag
        m_pipeline = TAGS_TO_PIPELINE[getTagNumber()];
        setPipeline();

        //Reset the PID controllers
		m_drivePID.reset();
		m_steerPID.reset();

        //Uncomment for Debug Purposes in Simulation
        //distErr = TY_MAX;
        //steerErr = TX_MAX;

        //Reset Failure Counts
        m_trackingStarted = true;
        m_targetLosses = 0;
        m_failureCount = 0;
        SmartDashboard.putBoolean("Red Alliance", IsRedAlliance.getBoolean(false));
    }

    public void InitializeAutonTracking(){
        if (IsRedAlliance.getBoolean(false)){
            m_pipeline = Constants.Limelight_Constants.RED_AUTO_PIPELINE;
        } else {
            m_pipeline =Constants.Limelight_Constants.BLUE_AUTO_PIPELINE;
        }
        setPipeline();

        //Reset the PID controllers
		m_drivePID.reset();
		m_steerPID.reset();

        //Reset Failure Counts
        m_trackingStarted = true;
        m_targetLosses = 0;
        m_failureCount = 0;
    }

    public void endTargetTracking(){
        //Set back to default pipeline
        m_pipeline = Constants.Limelight_Constants.DEFAULT_PIPELINE;
        setPipeline();

        m_trackingStarted = false;
    }
	
    public boolean atTarget(){
        //True if we have a target forward/backward and rotation are within tolerance
        //return targetAcquired() && m_drivePID.atSetpoint() && m_steerPID.atSetpoint();
        return m_drivePID.atSetpoint() && m_steerPID.atSetpoint() && m_trackingStarted;
    }

	public boolean calcMotorControl(){
        //Perform calculations if there is a target and at least one of the errors is not within tolerance
        //boolean validResult  = (targetAcquired() && (!m_drivePID.atSetpoint() || !m_steerPID.atSetpoint()));
        boolean validResult = !atTarget();
        //Make sure pipeline has been changed to the desired pipeline contained in m_pipeline
                //validResult  = validResult && (m_pipeline == getCurrentPipeline());

		double driveCommand  = 0.0;
		double steerCommand  = 0.0;
        double strafeCommand = 0.0;

        if (validResult){
            m_failureCount = MAX_FAILURES;//0;
            if (targetAcquired()){
                //Reset the intermittent Target Loss count
                m_targetLosses = 0;
			    //Calculate the next PID measurement for Driving ;
 			    if (!m_drivePID.atSetpoint()){
				    driveCommand = m_drivePID.calculate(distanceError());// / TY_MAX;
            	    if (Math.abs(driveCommand) < MIN_DRIVE){
					    driveCommand = Math.signum(driveCommand)*MIN_DRIVE; 
				    } else {
                        driveCommand = MathUtil.clamp(driveCommand, -MAX_DRIVE, MAX_DRIVE);
                    }
			    }
			
			    //Calculate the next PID measurement for Turning
			    if (!m_steerPID.atSetpoint()){
                    steerCommand = m_steerPID.calculate(steerError());// / TX_MAX;
                    if (Math.abs(steerCommand) < MIN_STEER) {
					    steerCommand = Math.signum(steerCommand)*MIN_STEER;
                    } else {
                        steerCommand = MathUtil.clamp(steerCommand, -MAX_STEER, MAX_STEER);
				    }

                    //If at correct distance, then use rotation and strafing
                    //if (m_drivePID.atSetpoint()){
                    //    //TODO need to verify if this is correct and if speed should be adjusted
                    //    strafeCommand = -steerCommand;
                    //    //steerCommand  =  steerCommand;
                    //} 
			    }
            } else {
                //Increment the consecutive target losses
                validResult = (m_targetLosses++ < MAX_TARGET_LOSSES);
            }
//		} else {
//			validResult = (++m_failureCount <= MAX_FAILURES);
		}
        
        m_driveCommand  = driveCommand;
        m_steerCommand  = steerCommand;
        m_strafeCommand = strafeCommand;

        SmartDashboard.putNumber("m_driveCommand", m_driveCommand);
        SmartDashboard.putNumber("m_steerCommand", m_steerCommand);
        SmartDashboard.putNumber("m_strafeCommand", m_strafeCommand);
        SmartDashboard.putBoolean("Drive at SP", m_drivePID.atSetpoint());
        SmartDashboard.putBoolean("Steer at SP", m_steerPID.atSetpoint());
        SmartDashboard.putNumber("TY", distanceError());//distErr);
        SmartDashboard.putNumber("TX", steerError());// steerErr);
        SmartDashboard.putBoolean("Target?", targetAcquired());
        SmartDashboard.putBoolean("At Target", atTarget());
        SmartDashboard.putNumber("Requested Pipeline", m_pipeline);
        SmartDashboard.putNumber("Actual Pipeline", getCurrentPipeline());
        SmartDashboard.putNumber("Tag Id", getTagNumber());
		return validResult;
	}

    /**************************************************************************************
      Instead of the following Altnernate calcMotorControl routines, you may
      need to calculate an Angle and Distance then write a completely drive
      command in SwerveDriveSubsystem to perform the following:
      1) Receives the current Heading of the Robot as a parameter
      2) Calculates the angle to turn to (0,90,180 or 270) based on Tag Id
                    the angle of wheels,
                    the distance to drive
      2) Write Methods to return each of the above values (see public void steerCommand())
      3) Use those values to drive
    ***************************************************************************************/ 
    public boolean calcMotorControlAlternate_1(){
        //Perform calculations if there is a target and at least one of the errors is not within tolerance
        /*************************************************************************
          Determination of validResult may have to change to include different PIDs
        **************************************************************************/
        boolean validResult  = (targetAcquired() && (!m_drivePID.atSetpoint() || !m_steerPID.atSetpoint()));
        //Make sure pipeline has been changed to the desired pipeline contained in m_pipeline
                validResult  = validResult && (m_pipeline == getCurrentPipeline());

		double driveCommand  = 0.0;
		double steerCommand  = 0.0;
        double strafeCommand = 0.0;

        if (validResult){
            m_failureCount = 0;
            /**********************************************************************
            Your Calculations for driveCommand  - Forward/Backward, Forward is +
                                  strafeCommand - Left/Right, Left is +
                                  steerCommand  - Rotation, CounterClockwise is + 
            go Here
            ***********************************************************************/
		} else {
			validResult = (++m_failureCount <= MAX_FAILURES);
		}
        
        m_driveCommand  = driveCommand;
        m_steerCommand  = steerCommand;
        m_strafeCommand = strafeCommand;

		return validResult;
    }

    public boolean calcMotorControlAlternate_2(){
        //Perform calculations if there is a target and at least one of the errors is not within tolerance
        /*************************************************************************
          Determination of validResult may have to change to include different PIDs
        **************************************************************************/
        boolean validResult  = (targetAcquired() && (!m_drivePID.atSetpoint() || !m_steerPID.atSetpoint()));
        //Make sure pipeline has been changed to the desired pipeline contained in m_pipeline
                validResult  = validResult && (m_pipeline == getCurrentPipeline());

		double driveCommand  = 0.0;
		double steerCommand  = 0.0;
        double strafeCommand = 0.0;

        if (validResult){
            m_failureCount = 0;
            /**********************************************************************
            Your Calculations for driveCommand  - Forward/Backward, Forward is +
                                  strafeCommand - Left/Right, Left is +
                                  steerCommand  - Rotation, CounterClockwise is + 
            go Here
            ***********************************************************************/
		} else {
			validResult = (++m_failureCount <= MAX_FAILURES);
		}
        
        m_driveCommand  = driveCommand;
        m_steerCommand  = steerCommand;
        m_strafeCommand = strafeCommand;

		return validResult;
    }
}

