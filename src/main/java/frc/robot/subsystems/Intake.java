/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Intake功能(靖凱)
//手動Lift:1RT/1LT
//單向Spin:1X 反向1B
//一鍵Encoder:

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;
//import frc.robot.Robot;
import frc.robot.Utility;

public class Intake extends SubsystemBase {

  private PIDController liftPID;
  private boolean liftPIDEnable = false;
  private double liftPIDSetpoint = 0;

  private WPI_VictorSPX liftMotor;
  private WPI_VictorSPX spinMotor;

  private Encoder liftEncoder;

  public Intake() {
    liftPID = new PIDController(Constants.intakeLiftPIDKp,Constants.intakeLiftPIDKi,Constants.intakeLiftPIDKd);
    liftMotor = new WPI_VictorSPX(Constants.intakeLiftMotorID);
    spinMotor = new WPI_VictorSPX(Constants.intakeSpinMotorID);
    liftMotor.setInverted(Constants.intakeLiftMotorInverted);
    spinMotor.setInverted(Constants.intakeSpinMotorInverted);
    liftEncoder = new Encoder(Constants.intakeLiftEncoderPinA, Constants.intakeLiftEncoderPinB, Constants.intakeLiftEncoderInverted);
    liftEncoder.reset();
  }

  @Override
  public void periodic() {
    if(liftPIDEnable){
      liftPIDOutput(liftPID.calculate(liftPIDMeasurment(), liftPIDSetpoint));
    }
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  public void setLiftMotorSpeed(double spd){
    liftMotor.set(spd);
  }
  public void setLiftMotorVoltage(double voltage){
    liftMotor.setVoltage(voltage);
  }
  public void setLiftMotorStop(){
    liftMotor.set(0);
  }

  public void setSpinMotorSpeed(double spd){
    spinMotor.set(spd);
  }
  public void setSpinMotorVoltage(double voltage){
    spinMotor.setVoltage(voltage);
  }
  public void setSpinMotorStop(){
    spinMotor.set(0);
  }

  public int getLiftEncoder(){
    return liftEncoder.get();
  }
  public void resetLiftEncoder(){
    liftEncoder.reset();
  }

  //Intake lift control PID function
  public void liftPIDEnable(){
    liftPIDEnable = true;
    liftPID.reset();
  }
  public void liftPIDDisable(){
    liftPIDEnable = false;
    liftPIDOutput(0);
  }
  public void liftPIDReset(){
    liftPIDSetpoint = 0;
    liftPID.reset();
  }
  public boolean liftPIDIsEnable(){
    return liftPIDEnable;
  }
  public void liftPIDSetSetpoint(double setpoint){
    liftPIDSetpoint = setpoint;
  }
  public void liftPIDSetTolerance(double value){
    liftPID.setTolerance(value);
  }
  public double liftPIDMeasurment(){
    return getLiftEncoder();
  }
  public void liftPIDOutput(double output){
    double voltage = output;
    voltage = Utility.Constrain(voltage, -Constants.intakeLiftPIDMaxOutputVoltage, Constants.intakeLiftPIDMaxOutputVoltage);
    setLiftMotorVoltage(voltage);
  }
  public boolean liftPIDIsStable(){
    return liftPID.atSetpoint();
  }
}
