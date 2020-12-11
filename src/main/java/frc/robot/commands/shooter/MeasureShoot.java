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

public class MeasureShoot extends CommandBase {

  private int buttonYStatus = 0;
  private int buttonBStatus = 0;
  private int buttonXStatus = 0;
  private int buttonAStatus = 0;

  private double upperVoltage = 0;
  private double lowerVoltage = 0;

  private final Shooter shooter;
  public MeasureShoot(Shooter subsystem) {
    shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    shooter.upperPIDDisable();
    shooter.lowerPIDDisable();
  }

  @Override
  public void execute() {
    if(Robot.m_oi.getARawButton(Constants.buttonY) && buttonYStatus == 0){
      upperVoltage += 0.5;
      buttonYStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonY) && buttonYStatus == 1){
      buttonYStatus = 0;
    }
    if(Robot.m_oi.getARawButton(Constants.buttonB) && buttonBStatus == 0){
      upperVoltage -= 0.5;
      buttonBStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonB) && buttonBStatus == 1){
      buttonBStatus = 0;
    }

    if(Robot.m_oi.getARawButton(Constants.buttonX) && buttonXStatus == 0){
      lowerVoltage += 0.5;
      buttonXStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonX) && buttonXStatus == 1){
      buttonXStatus = 0;
    }
    if(Robot.m_oi.getARawButton(Constants.buttonA) && buttonAStatus == 0){
      lowerVoltage -= 0.5;
      buttonAStatus = 1;
    }else if(!Robot.m_oi.getARawButton(Constants.buttonA) && buttonAStatus == 1) {
      buttonAStatus = 0;
    }

    upperVoltage = Utility.Constrain(upperVoltage, 0, 10);
    lowerVoltage = Utility.Constrain(lowerVoltage, 0, 10);

    SmartDashboard.putNumber("UpperVoltage", upperVoltage);
    SmartDashboard.putNumber("LowerVoltage", lowerVoltage);
    
    /*
    double upperSpeed = Robot.m_oi.getRawAxis(Constants.axisRT);
    double lowerSpeed = Robot.m_oi.getRawAxis(Constants.axisLT);
    SmartDashboard.putNumber("upperSpeed", upperSpeed);
    SmartDashboard.putNumber("lowerSpeed", lowerSpeed);
    shooter.setUpperSpeed(upperSpeed);
    shooter.setLowerSpeed(lowerSpeed);
    SmartDashboard.putNumber("upperNowSpeed", shooter.getUpperPIDMeasurment());
    SmartDashboard.putNumber("lowerNowSpeed", shooter.getLowerPIDMeasurment());
    SmartDashboard.putBoolean("Status", true);
    */
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setUpperMotorSpeed(0);
    shooter.setLowerMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
