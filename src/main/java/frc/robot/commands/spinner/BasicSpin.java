/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Spinner;

public class BasicSpin extends CommandBase {

  private final Spinner spinner;
  public BasicSpin(Spinner subsystem) {
    spinner = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
    if(Robot.m_oi.getARawButton(Constants.buttonY)){//單向Spin
      spinner.setLiftMotorSpeed(1);
    }else if(Robot.m_oi.getARawButton(Constants.buttonA)){
      spinner.setLiftMotorSpeed(-1);
    }else{
      spinner.setLiftMotorSpeed(0);
    }

    if(Robot.m_oi.getARawButton(Constants.buttonLB)){
      spinner.setSpinMotorSpeed(1);
    }else if(Robot.m_oi.getARawButton(Constants.buttonRB)){
      spinner.setSpinMotorSpeed(-1);
    }else{
      spinner.setSpinMotorSpeed(0);
    }
    
  }
  @Override
  public void end(boolean interrupted) {
    spinner.setLiftMotorStop();
    spinner.setSpinMotorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
