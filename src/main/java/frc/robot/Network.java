/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Network {
    
    public Network(){

    }

    public boolean ntGetBoolean(String tableName, String keyName){
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable(tableName);
      NetworkTableEntry data = table.getEntry(keyName);
      boolean result = data.getBoolean(false);
      return result;
    }
    public double ntGetDouble(String tableName, String keyName){
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable(tableName);
      NetworkTableEntry data = table.getEntry(keyName);
      double result = data.getDouble(0.0);
      return result;
    }
}
