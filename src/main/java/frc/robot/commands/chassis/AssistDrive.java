/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utility;
import frc.robot.subsystems.Chassis;

public class AssistDrive extends CommandBase {

  private Timer timer;

  private boolean disable = false;
  private double previousTime = 0;

  Chassis chassis;
  public AssistDrive(Chassis subsystem) {
    chassis = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    chassis.setLockAngle(chassis.getRawAngle());
    chassis.headingPIDEnable();
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if(disable){
      chassis.setLockAngle(chassis.getRawAngle());
      chassis.headingPIDEnable();
      disable = false;
    }

    double joystickLY = Robot.m_oi.getALY();
    double joystickRX = Robot.m_oi.getARX();

    if(Robot.m_oi.isARXDeadzone()){
      if(chassis.headingPIDIsEnable()){
      }else{
        if(timer.get() >= previousTime + Constants.chassisHeadingPIDRestartTime){
          chassis.setLockAngle(chassis.getRawAngle());
          chassis.headingPIDEnable();
        }
        double Lspd = Utility.Constrain(joystickLY, -1, 1);
        double Rspd = Utility.Constrain(joystickLY, -1, 1);
        chassis.setMotorSpeed(Lspd, Rspd);
      }
    }else{
      chassis.headingPIDDisable();
      previousTime = timer.get();
      double Lspd = Utility.Constrain((joystickLY + joystickRX), -1, 1);
      double Rspd = Utility.Constrain((joystickLY - joystickRX), -1, 1);
      chassis.setMotorSpeed(Lspd, Rspd);
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.headingPIDDisable();
    chassis.setMotorStop();
    disable = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
