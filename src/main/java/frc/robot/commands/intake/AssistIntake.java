/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class AssistIntake extends CommandBase {

  private boolean disable = false;
  
  Intake intake;
  public AssistIntake(Intake subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    intake.resetLiftEncoder();
    intake.liftPIDEnable();
  }

  @Override
  public void execute() {
    if(disable){
      intake.liftPIDEnable();
      disable = false;
    }
    
    if(Robot.m_oi.getARawButton(Constants.buttonA)){
      intake.setSpinMotorSpeed(0.5);
    }else{
      intake.setSpinMotorStop();
    }

    if(Robot.m_oi.getARawButton(Constants.buttonY)){
      intake.liftPIDSetSetpoint(angleToPulse(0));
    }else if(Robot.m_oi.getARawButton(Constants.buttonA)){
      intake.liftPIDSetSetpoint(angleToPulse(90));
    }

  }

  @Override
  public void end(boolean interrupted) {
    intake.liftPIDDisable();
    intake.setLiftMotorStop();
    intake.setSpinMotorStop();
    disable = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double angleToPulse(double angle) {
    return (int)((angle / 360) * (Constants.intakeLiftMotorGearRatio * 7));
  }
}
