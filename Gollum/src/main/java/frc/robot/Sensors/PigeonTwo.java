package frc.robot.Sensors;

//import com.ctre.phoenix6.ErrorCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
//FIXME import com.ctre.phoenix6.configs.Pigeon2_Faults;
//FIXME import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
/**
 * Class obtained from Hatter's Robotics, FRC Team #708
 *      Updated by #6808 for Phoenix 6 API Library 2024 Season
 */
public class PigeonTwo {
    
    //Pigeon instance (ensure it is a singleton)
    private static PigeonTwo instance = null;

    /**
     * Method to get current instance of Pigeon2 (or establish one if one does not exist)
     * @return Pigeon2 Instance
     */
    public static PigeonTwo getInstance(){
        if(instance == null){
            instance = new PigeonTwo();
        }
        return instance;
    }

    //Pigeon2 object
    private Pigeon2 m_pigeon2;

    //Pigeon2 configuration settings
    private Pigeon2Configuration config = new Pigeon2Configuration();

    //FIXME Pigeon2 Fault Log
    //private Pigeon2_Faults faultLog = new Pigeon2_Faults();

    //FIXME Fault ErrorCode object
    //ErrorCode faults;

    //GravityVector object
    private double[] a_gravityVector = new double[3];
    
    private PigeonTwo(){
        try{
            m_pigeon2 = new Pigeon2(Constants.SwerveDriveConstants.PIGEON_CANID);
            //Sets the status frame period for two different periods.
            //Can't find Phoenix 6 equivalent
            //m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, 10);
            //m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 5, 10);
            
            //Default configs = MAKE CONSTANTS IN CONSTANTS FILE & DETERMINE VALUES
            config.Pigeon2Features.DisableNoMotionCalibration = false;
            config.Pigeon2Features.DisableTemperatureCompensation = false;
            config.Pigeon2Features.EnableCompass = true;
            config.MountPose.MountPosePitch = 0;
            config.MountPose.MountPoseRoll  = 0;
            config.MountPose.MountPoseYaw   = 0;

            //Sets pigeon default mountings to values determined above
            m_pigeon2.getConfigurator().apply(config); 

            //Gets gravity vectory and assigns it to a_gravityVector
            //m_pigeon2.getGravityVector(a_gravityVector);
            a_gravityVector[0] = m_pigeon2.getGravityVectorX().getValueAsDouble();
            a_gravityVector[1] = m_pigeon2.getGravityVectorY().getValueAsDouble();
            a_gravityVector[2] = m_pigeon2.getGravityVectorZ().getValueAsDouble();
            

            //FIXME Assigns faultLog to record errors
            //faults = m_pigeon2.getFaults(faultLog);
        }catch(Exception e){
            System.out.println("PIGEON INSTANTATION FAILED");
            //Can not find Phoexix 6 Equivalent
            //m_pigeon2.DestroyObject();
            e.printStackTrace();
        }
    }

    // public boolean isGood(){
    //     return true;
    // }

    public Rotation2d getPitch(){
        double pitch = m_pigeon2.getPitch().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(pitch);
    }

    public Rotation2d getAngle(){
//        double yaw = m_pigeon2.getYaw().refresh().getValueAsDouble();
//        return Rotation2d.fromDegrees(yaw);
        return Rotation2d.fromDegrees(-m_pigeon2.getAngle());
    }

    public Rotation2d getRoll(){
        double roll = m_pigeon2.getRoll().refresh().getValueAsDouble(); //COMPARE TO getYPR();
        return Rotation2d.fromDegrees(roll);
    }

    public double getRateX(){
        return m_pigeon2.getAngularVelocityXDevice().refresh().getValueAsDouble();
    }

    public double getRateY(){
       return m_pigeon2.getAngularVelocityYDevice().refresh().getValueAsDouble();
    }

    public double getRateZ(){
        //return m_pigeon2.getAngularVelocityZDevice().getValueAsDouble();
        return m_pigeon2.getRate();
    }

    public double getRawAngle(){
        return m_pigeon2.getYaw().refresh().getValueAsDouble();
    }

    public double get360Angle(){
        //return m_pigeon2.getYaw().refresh().getValueAsDouble() % 360.0;
        return -m_pigeon2.getAngle() % 360.0;
    }

    public StatusCode setAngle(double degrees){
        return m_pigeon2.setYaw(-degrees);
    }

    public StatusCode reset(){
        return setAngle(0);
    }

    public void getGravityVector(){
        a_gravityVector[0] = m_pigeon2.getGravityVectorX().refresh().getValueAsDouble();
        a_gravityVector[1] = m_pigeon2.getGravityVectorY().refresh().getValueAsDouble();
        a_gravityVector[2] = m_pigeon2.getGravityVectorZ().refresh().getValueAsDouble();
        //return a_gravityVector;
    }

//FIXME CODE RELATED TO ERROR MANAGEMENT//
/*
    public ErrorCode getLastError(){
        return m_pigeon2.getLastError();
    }

    public boolean getAPIError(){
        return faultLog.APIError;
    }

    public boolean getAccelError(){
        return faultLog.AccelFault;
    }

    public boolean getBootIntoMotionError(){
        return faultLog.BootIntoMotion;
    }

    public boolean getGyroError(){
        return faultLog.GyroFault;
    }

    public boolean getHardwareError(){
        return faultLog.HardwareFault;
    }

    public boolean getMagnetometerError(){
        return faultLog.MagnetometerFault;
    }

    public boolean getMotionDriverError(){
        return faultLog.BootIntoMotion;
    }

    public boolean getResetError(){
        return faultLog.ResetDuringEn;
    }

    public boolean getSaturatedAccelError(){
        return faultLog.SaturatedAccel;
    }

    public boolean getSaturatedMagError(){
        return faultLog.SaturatedMag;
    }

    public boolean getSaturatedRotVelError(){
        return faultLog.SaturatedRotVelocity;
    }

    public boolean getVoltageError(){
        return faultLog.UnderVoltage;
    }
*/
}
