/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveAngle extends CommandBase {

  boolean disable = false;
  boolean ignore = true;
  double targetAngle = 0;

  Chassis chassis;
  public DriveAngle(Chassis subsystem,double turnAngle) {
    chassis = subsystem;
    addRequirements(subsystem);
    targetAngle = turnAngle;
    chassis.turnPIDSetTolerance(1);
    chassis.turnPIDSetSetpoint(0);
  }

  @Override
  public void initialize() {
    chassis.setLockAngle(chassis.getRawAngle() + targetAngle);
    chassis.turnPIDEnable();
  }

  @Override
  public void execute() {
    if(disable){
      //chassis.setLockAngle(chassis.getRawAngle() + targetAngle);
      chassis.turnPIDEnable();
      disable = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.turnPIDDisable();
    chassis.setMotorStop();
    disable = true;
    ignore = true;
  }

  @Override
  public boolean isFinished() {
    boolean finished = chassis.turnPIDIsStable();
    if(finished && ignore){
      ignore = false;
      return false;
    }else if(finished && !ignore){
      return true;
    }else{
      return false;
    }
  }
}
