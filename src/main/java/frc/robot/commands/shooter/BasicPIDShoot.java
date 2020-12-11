/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utility;
import frc.robot.subsystems.Shooter;

public class BasicPIDShoot extends CommandBase {

  private double upperRPS = 20;
  private double lowerRPS = 20;
  private int buttonYStatus = 0;
  private int buttonBStatus = 0;
  private int buttonXStatus = 0;
  private int buttonAStatus = 0;
  
  private final Shooter shooter;
  public BasicPIDShoot(Shooter subsystem) {
    shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    shooter.upperPIDEnable();
    shooter.lowerPIDEnable();
  }

  @Override
  public void execute(){
    if(Robot.m_oi.getARawButton(Constants.buttonY) && buttonYStatus == 0){
      upperRPS += 2.5;
      buttonYStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonY) && buttonYStatus == 1){
      buttonYStatus = 0;
    }
    if(Robot.m_oi.getARawButton(Constants.buttonB) && buttonBStatus == 0){
      upperRPS -= 2.5;
      buttonBStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonB) && buttonBStatus == 1){
      buttonBStatus = 0;
    }

    if(Robot.m_oi.getARawButton(Constants.buttonX) && buttonXStatus == 0){
      lowerRPS += 2.5;
      buttonXStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonX) && buttonXStatus == 1){
      buttonXStatus = 0;
    }
    if(Robot.m_oi.getARawButton(Constants.buttonA) && buttonAStatus == 0){
      lowerRPS -= 2.5;
      buttonAStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonA) && buttonAStatus == 1) {
      buttonAStatus = 0;
    }

    upperRPS = Utility.Constrain(upperRPS, 0, Constants.shooterUpperMotorMaxPIDRPS);
    lowerRPS = Utility.Constrain(lowerRPS, 0, Constants.shooterLowerMotorMaxPIDRPS);

    SmartDashboard.putNumber("UpperTargetRPS", upperRPS);
    SmartDashboard.putNumber("LowerTargetRPS", lowerRPS);

    if(upperRPS > 0){
      if(!shooter.upperPIDIsEnable()){
        shooter.upperPIDEnable();
      }
      shooter.setUpperPIDSetpoint(upperRPS);
    }else{
      shooter.upperPIDDisable();
      shooter.upperPIDReset();
    }

    if(lowerRPS > 0){
      if(!shooter.lowerPIDIsEnable()){
        shooter.lowerPIDEnable();
      }
      shooter.setLowerPIDSetpoint(lowerRPS);
    }else{
      shooter.lowerPIDDisable();
      shooter.lowerPIDReset();
    }

    SmartDashboard.putBoolean("upperPID", shooter.upperPIDIsEnable());
    SmartDashboard.putBoolean("lowerPID", shooter.lowerPIDIsEnable());

    SmartDashboard.putNumber("UpperNowRPS", shooter.getUpperPIDMeasurment());
    SmartDashboard.putNumber("LowerNowRPS", shooter.getLowerPIDMeasurment());
    SmartDashboard.putBoolean("status", true);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("status", false);
    shooter.upperPIDDisable();
    shooter.lowerPIDDisable();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
