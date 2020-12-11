/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveTime extends CommandBase {
  
  private Timer timer;

  private double time = 0;
  private double Lspd = 0;
  private double Rspd = 0;

  Chassis chassis;
  public DriveTime(Chassis subsystem, Double time, Double Lspd, Double Rspd) {
    System.out.print("Here");
    chassis = subsystem;
    addRequirements(subsystem);
    this.time = time; 
    this.Lspd = Lspd;
    this.Rspd = Rspd;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    System.out.println(timer.get());
    chassis.setLeftMotorSpeed(Lspd);
    chassis.setRightMotorSpeed(Rspd);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setMotorStop();
  }

  @Override
  public boolean isFinished() {
    return (timer.get() > time);
  }
}
