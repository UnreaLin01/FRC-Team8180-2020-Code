/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Utility {
    public static double Constrain(double value,double min,double max){
        if(value > max){
          value = max;
        }else if(value < min){
          value = min;
        }
        return value;
      }

    public static double Map(double value,double inMin,double inMax,double outMin,double outMax){
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
