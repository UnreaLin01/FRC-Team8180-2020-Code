/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;

public class DriveDistance extends CommandBase {
  
  private boolean disable = false;
  private boolean ignore = true;

  Chassis chassis;
  public DriveDistance(Chassis subsystem, double LeftDistance, double RightDistance) {
    chassis = subsystem;
    addRequirements(subsystem);
    chassis.LMotorPIDSetTolerance(turnsToPulse(0.2));
    chassis.RMotorPIDSetTolerance(turnsToPulse(0.2));
    chassis.LMotorPIDSetSetpoint(distaceToPulse(LeftDistance));
    chassis.RMotorPIDSetSetpoint(distaceToPulse(RightDistance));
  }

  @Override
  public void initialize() {
    chassis.LMotorPIDEnable();
    chassis.RMotorPIDEnable();
    
  }

  @Override
  public void execute() {
    if(disable){
      chassis.LMotorPIDEnable();
      chassis.RMotorPIDEnable();
      disable = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.LMotorPIDDisable();
    chassis.RMotorPIDDisable();
    chassis.setLeftMotorStop();
    chassis.setRightMotorStop();
    disable = true;
    ignore = true;
  }

  @Override
  public boolean isFinished() {
    boolean finished = chassis.LMotorPIDIsStable() && chassis.RMotorPIDIsStable();
    if(finished && ignore){
      ignore = false;
      return false;
    }else if(finished && !ignore){
      return true;
    }else{
      return false;
    }
  }

  public double distaceToPulse(double distanceMeter){
    return ((distanceMeter / Constants.chassisWheelPerimeter) * 4096.0);
  }

  public double turnsToPulse(double turns){
    return (turns * 4096.0); 
  }
}
