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

public class BasicIntake extends CommandBase {

  private final Intake intake;
  public BasicIntake(Intake subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double ART = Robot.m_oi.getARawAxis(Constants.axisRT);
    double ALT = Robot.m_oi.getARawAxis(Constants.axisLT);
    
    if(Robot.m_oi.getARawButton(Constants.buttonX)){
      intake.setSpinMotorSpeed(0.5);
    }else if(Robot.m_oi.getARawButton(Constants.buttonB)){
      intake.setSpinMotorSpeed(-0.5);
    }else{
      intake.setSpinMotorSpeed(0);
    }

    if(ART > Constants.joystickDeadZone){
      intake.setLiftMotorSpeed(ART);
    }else if(ALT > Constants.joystickDeadZone){
      intake.setLiftMotorSpeed(ALT);
    }else{
      intake.setLiftMotorSpeed(0);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    intake.setLiftMotorStop();
    intake.setSpinMotorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
