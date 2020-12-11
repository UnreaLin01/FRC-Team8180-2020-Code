/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*///Climber(鵬嘉)
//hook:2LY
//hanger:2RY
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
//import frc.robot.Robot;

public class Climber extends SubsystemBase {

  private WPI_TalonSRX hookF;
  private WPI_TalonSRX hookB;
  private WPI_TalonSRX lift;

  //private DigitalInput liftTopLimitSw;
  //private DigitalInput liftBottomLimitSw;

  public Climber() {
    
    hookF = new WPI_TalonSRX(Constants.climberHookFrontMotorID);
    hookB = new WPI_TalonSRX(Constants.climberHookBackMotorID);
    lift = new WPI_TalonSRX(Constants.climberLiftMotorID);
    
    //liftTopLimitSw = new DigitalInput(Constants.climberLiftTopLimitSwID);
    //liftBottomLimitSw = new DigitalInput(Constants.climberLiftBottomLimitSwID);

    hookF.setInverted(Constants.climberHookMotorInverted);
    lift.setInverted(Constants.climberLiftMotorInverted);
    hookB.follow(hookF);

    
    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  //Hook motor function
  public void setHookMotorSpeed(double spd){
    hookF.set(spd);
  }
  public void setHookMotorStop(){
    hookF.set(0);
  }

  //Hanger motor function
  public void setLiftMotorSpeed(double spd){
    //LimitSw Protect Feature
    /*
    if(liftTopLimitSw.get() || liftBottomLimitSw.get()){
      lift.set(0);
    }else{
      lift.set(spd);
    }
    */
    
    lift.set(spd);
  }
  public void setLiftMotorStop(){
    lift.set(0);
  }
/*
  public boolean getLiftTopLimitSw(){
    return liftTopLimitSw.get();
  }
  public boolean getLiftBottomLimitSw(){
    return liftBottomLimitSw.get();
  }
*/
}
