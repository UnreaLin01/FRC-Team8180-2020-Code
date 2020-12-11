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

public class AimDrive extends CommandBase {
  
  private boolean disable = false;
  private boolean ignore = true;

  Chassis chassis;
  public AimDrive(Chassis subsystem) {
    chassis = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    chassis.visionPIDSetTolerance(1);
    chassis.visionPIDEnable();
  }

  @Override
  public void execute(){
    if(disable){
      chassis.visionPIDEnable();
      disable = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.visionPIDDisable();
    chassis.setMotorStop();
    disable = true;
    ignore = true;
  }

  @Override
  public boolean isFinished() {
    boolean finished = chassis.headingPIDIsStable();
    if(finished && ignore){
      ignore = false;
      return false;
    }else if(finished && !ignore){
      return true;
    }
    return false;
  }
}
