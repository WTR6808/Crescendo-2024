// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class FieldManagementSystem {
    private NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
    private NetworkTableEntry m_eventName = FMSInfo.getEntry("EventName");
    private NetworkTableEntry m_gameSpecificMessage = FMSInfo.getEntry("GameSpecificMessage");
    private NetworkTableEntry m_isRedAlliance  = FMSInfo.getEntry("IsRedAlliance");
    private NetworkTableEntry m_matchNumber = FMSInfo.getEntry("MatchNumber");
    private NetworkTableEntry m_matchType = FMSInfo.getEntry("MatchType");
    private NetworkTableEntry m_replayNumber = FMSInfo.getEntry("ReplayNumber");
    private NetworkTableEntry m_stationNumber = FMSInfo.getEntry("StationNumber");

    //FMS instance (ensure it is a singleton)
    private static FieldManagementSystem instance = null;

    /**
     * Method to get current instance of Pigeon2 (or establish one if one does not exist)
     * @return Pigeon2 Instance
     */
    public static FieldManagementSystem getInstance(){
        if(instance == null){
            instance = new FieldManagementSystem();
        }
        return instance;
    }

    private FieldManagementSystem(){

    }

    public String getEventName (){
        return m_eventName.getString("");
    }

    public String getGameSpecificMessage(){
        return m_gameSpecificMessage.getString("");
    }

    public Boolean isRedAlliance(){
        return m_isRedAlliance.getBoolean(true);
    }

    public int getMatchNumber(){
        return (int) m_matchNumber.getInteger(0);
    }

    public int getMatchType(){
        return (int) m_matchType.getInteger(0);
    }

    public int getReplayNumber(){
        return (int) m_replayNumber.getInteger(0);
    }

    public int getStationNumber(){
        return (int) m_stationNumber.getInteger(0);
    }
}
