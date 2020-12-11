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

public class AssistSpin extends CommandBase {

  private boolean disable = false;
  
  Spinner spinner;
  public AssistSpin(Spinner subsystem) {
    spinner = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    spinner.resetLiftEncoder();
    spinner.liftPIDEnable();
  }

  @Override
  public void execute() {
    if(disable){
      spinner.liftPIDEnable();
      disable = false;
    }

    if(Robot.m_oi.getARawButton(Constants.buttonA)){
      spinner.setSpinMotorSpeed(0.5);
    }else{
      spinner.setSpinMotorStop();
    }
    
    if(Robot.m_oi.getARawButton(Constants.buttonY)){
      spinner.liftPIDSetSetpoint(angleToPulse(0));
    }else if(Robot.m_oi.getARawButton(Constants.buttonA)){
      spinner.liftPIDSetSetpoint(angleToPulse(90));
    }
  }

  @Override
  public void end(boolean interrupted) {
    spinner.liftPIDDisable();
    spinner.setLiftMotorStop();
    spinner.setSpinMotorStop();
    disable = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double angleToPulse(double angle){
    return (int)((angle / 360) * (Constants.spinnerLiftMotorGearRatio * 7));
  }
}
