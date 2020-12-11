/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.Utility;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends CommandBase {

  private final Shooter shooter;
  public BasicShoot(Shooter subsystem) {
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
    double BLT = Robot.m_oi.getBRawAxis(Constants.axisLT);
  
    if(Math.abs(BLT) > Constants.joystickDeadZone){
      shooter.setFeedMotorSpeed(0.3);
      shooter.setUpperMotorSpeed(1);
      shooter.setLowerMotorSpeed(1);
    }else{
      shooter.setFeedMotorSpeed(0);
      shooter.setUpperMotorSpeed(0);
      shooter.setLowerMotorSpeed(0);     
    }

    
    double ALX = Robot.m_oi.getARawAxis(Constants.axisJLX);
    if(Math.abs(ALX) > Constants.joystickDeadZone){
      if(ALX > 1){
        shooter.setContainerMotorSpeed(1);
      }else if(ALX < 1){
        shooter.setContainerMotorSpeed(-1);
      }
    }else{
      shooter.setContainerMotorStop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setContainerMotorStop();
    shooter.setFeedMotorStop();
    shooter.setUpperMotorStop();
    shooter.setLowerMotorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
