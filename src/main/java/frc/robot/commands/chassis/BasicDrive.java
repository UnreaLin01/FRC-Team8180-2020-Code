/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Utility;
import frc.robot.subsystems.Chassis;

public class BasicDrive extends CommandBase {
  private final Chassis chassis;
  public BasicDrive(Chassis subsystem) {
    chassis = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double Rspd = Robot.m_oi.getALY() - Robot.m_oi.getARX();
    double Lspd = Robot.m_oi.getALY() + Robot.m_oi.getARX();
    Rspd = Utility.Constrain(Rspd, -1, 1);
    Lspd = Utility.Constrain(Lspd, -1, 1);
    chassis.setMotorSpeed(Lspd, Rspd);
  }
  
  @Override
  public void end(boolean interrupted) {
    chassis.setMotorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
