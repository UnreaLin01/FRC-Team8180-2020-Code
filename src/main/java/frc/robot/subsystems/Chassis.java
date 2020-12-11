/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//BasicDrive:
//drive:1LY 1RX

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Network;
import frc.robot.Robot;
import frc.robot.Utility;


public class Chassis extends SubsystemBase {
  private PIDController headingPID;
  private boolean headingPIDEnable = false;
  private double headingPIDSetpoint = 0;

  private PIDController turnPID;
  private boolean turnPIDEnable = false;
  private double turnPIDSetpoint = 0;

  private PIDController visionPID;
  private boolean visionPIDEnable = false;
  private double visionPIDSetpoint = 0;

  private PIDController LMotorPID;
  private boolean LMotorPIDEnable = false;
  private double LMotorPIDSetpoint = 0;

  private PIDController RMotorPID;
  private boolean RMotorPIDEnable = false;
  private double RMotorPIDSetpoint = 0;

  //private Timer timer;
  private AHRS navx;
  private Network network;

  private WPI_TalonSRX motorRF;
  private WPI_TalonSRX motorRB;
  private WPI_TalonSRX motorLF;
  private WPI_TalonSRX motorLB;

  private double lockAngle = 0;

  public Chassis() {
    headingPID = new PIDController(Constants.chassisHeadingPIDKp, Constants.chassisHeadingPIDKi, Constants.chassisHeadingPIDKd);
    turnPID = new PIDController(Constants.chassisTurnPIDKp, Constants.chassisTurnPIDKi, Constants.chassisTurnPIDKd);
    visionPID = new PIDController(Constants.chassisVisionPIDKp, Constants.chassisVisionPIDKi, Constants.chassisVisionPIDKd);
    LMotorPID = new PIDController(Constants.chassisMotorPIDKp, Constants.chassisMotorPIDKi, Constants.chassisMotorPIDKd);
    RMotorPID = new PIDController(Constants.chassisMotorPIDKp, Constants.chassisMotorPIDKi, Constants.chassisMotorPIDKd);

    motorRF = new WPI_TalonSRX(Constants.chassisMotorRFID);
    motorRB = new WPI_TalonSRX(Constants.chassisMotorRBID);
    motorLF = new WPI_TalonSRX(Constants.chassisMotorLFID);
    motorLB = new WPI_TalonSRX(Constants.chassisMotorLBID);
    motorRF.setInverted(Constants.chassisMotorRInverted);
    motorLF.setInverted(Constants.chassisMotorLInverted);
    motorRB.follow(motorRF);
    motorLB.follow(motorLF);

    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    network = new Network();
  }

  @Override
  public void periodic() {

    /*
    //Used by "AssistDrive"
    if(headingPIDEnable){
      headingPIDOutput(headingPID.calculate(headingPIDMeasurment(), headingPIDSetpoint));
    }

    //Use by "AimDrive"
    if(visionPIDEnable){
      visionPIDOutput(visionPID.calculate(visionPIDMeasurment(), visionPIDSetpoint));
    }

    //Used by "DriveDistance"
    if(LMotorPIDEnable){
      LMotorPIDOutput(LMotorPID.calculate(LMotorPIDMeasurment(), LMotorPIDSetpoint));
    }
    if(RMotorPIDEnable){
      RMotorPIDOutput(RMotorPID.calculate(RMotorPIDMeasurment(), RMotorPIDSetpoint));
    }
    //Used by "DriveAngle"
    if(turnPIDEnable){
      turnPIDOutput(turnPID.calculate(turnPIDMeasurment(), turnPIDSetpoint));
    }
    */

  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }


  //Motor control function
  public void setMotorSpeed(double Lspd, double Rspd){
    if(Robot.m_oi.getARawButton(Constants.buttonOption)){
      motorLF.set(Lspd * Constants.chassisMotorSlowModeSpeedScale);
      motorRF.set(Rspd * Constants.chassisMotorSlowModeSpeedScale);
    }else{
      motorLF.set(Lspd * Constants.chassisMotorNormalModeSpeedScale);
      motorRF.set(Rspd * Constants.chassisMotorNormalModeSpeedScale);
    }
  }
  public void setMotorVoltage(double Lvoltage, double Rvoltage){
    motorLF.setVoltage(Lvoltage);
    motorRF.setVoltage(Rvoltage);
  }
  public void setMotorStop(){
    setMotorSpeed(0, 0);
  }

  public void setLeftMotorSpeed(double spd){
    if(Robot.m_oi.getARawButton(Constants.buttonOption)){
      motorLF.set(spd * Constants.chassisMotorSlowModeSpeedScale);
    }else{
      motorLF.set(spd * Constants.chassisMotorNormalModeSpeedScale);
    }
  }
  public void setLeftMotorVoltage(double voltage){
    motorLF.setVoltage(voltage);
  }
  public void setLeftMotorStop(){
    setLeftMotorSpeed(0);
  }

  public void setRightMotorSpeed(double spd){
    if(Robot.m_oi.getARawButton(Constants.buttonOption)){
      motorRF.set(spd * Constants.chassisMotorSlowModeSpeedScale);
    }else{
      motorRF.set(spd * Constants.chassisMotorNormalModeSpeedScale);
    }
  }
  public void setRightMotorVoltage(double voltage){
    motorRF.setVoltage(voltage);
  }
  public void setRightMotorStop(){
    setRightMotorSpeed(0);
  }
  
  //Motor Sensor function
  public double getRightMotorEncoder(){
    return (motorRF.getSensorCollection().getPulseWidthPosition());
  }
  public double getLeftMotorEncoder(){
    return (motorLF.getSensorCollection().getPulseWidthPosition());
  }


  //NavX function
  public double getRawAngle(){
    return navx.getAngle() % 360;
  }
  public double getCalAngle(){
    return (((((getRawAngle() + (360 - lockAngle)) % 360) + 180) % 360) - 180);
  }
  public void setLockAngle(double angle){
    lockAngle = angle;
  }


  //Heading control PID function
  public void headingPIDEnable(){
    headingPIDEnable = true;
    headingPID.reset();
  }
  public void headingPIDDisable(){
    headingPIDEnable = false;
    headingPIDOutput(0);
  }
  public void headingPIDReset(){
    headingPIDSetpoint = 0;
    headingPID.reset();
  }
  public boolean headingPIDIsEnable(){
    return headingPIDEnable;
  }
  public void headingPIDSetSetpoint(double setpoint){
    headingPIDSetpoint = setpoint;
  }
  public void headingPIDSetTolerance(double value){
    headingPID.setTolerance(value);
  }
  public double headingPIDMeasurment(){
    return getCalAngle();
  }
  public void headingPIDOutput(double output){
    double Rspd = Robot.m_oi.getALY() - output;
    double Lspd = Robot.m_oi.getALY() + output;
    setMotorSpeed(Lspd, Rspd);
  }
  public boolean headingPIDIsStable(){
    return headingPID.atSetpoint();
  }


  //Turn control PID function
  public void turnPIDEnable(){
    turnPIDEnable = true;
    turnPID.reset();
  }
  public void turnPIDDisable(){
    turnPIDEnable = false;
    turnPIDOutput(0);
  }
  public void turnPIDReset(){
    turnPIDSetpoint = 0;
    turnPID.reset();
  }
  public boolean turnPIDIsEnable(){
    return turnPIDEnable;
  }
  public void turnPIDSetSetpoint(double setpoint){
    turnPIDSetpoint = setpoint;
  }
  public void turnPIDSetTolerance(double value){
    turnPID.setTolerance(value);
  }
  public double turnPIDMeasurment(){
    return getCalAngle();
  }
  public void turnPIDOutput(double output){
    double Rvoltage = output;
    double Lvoltage = output;
    Rvoltage = Utility.Constrain(Rvoltage, -Constants.chassisTurnPIDMaxOutputVoltage, Constants.chassisTurnPIDMaxOutputVoltage);
    Lvoltage = Utility.Constrain(Lvoltage, -Constants.chassisTurnPIDMaxOutputVoltage, Constants.chassisTurnPIDMaxOutputVoltage);
    setMotorSpeed(Lvoltage, Rvoltage);
  }
  public boolean turnPIDIsStable(){
    return turnPID.atSetpoint();
  }
  

  //Vision PID function
  public void visionPIDEnable(){
    visionPIDEnable = true;
    visionPID.reset();
  }
  public void visionPIDDisable(){
    visionPIDEnable = false;
    visionPIDOutput(0);
  }
  public void visionPIDReset(){
    visionPID.reset();
  }
  public boolean visionPIDIsEnable(){
    return visionPIDEnable;
  }
  public void visionPIDSetSetpoint(double setpoint){
    visionPIDSetpoint = setpoint;
  }
  public void visionPIDSetTolerance(double value){
    visionPID.setTolerance(value);
  }
  public double visionPIDMeasurment(){
    return network.ntGetDouble("Vision", "h_angle");
  }
  public void visionPIDOutput(double output){
    double Rvoltage = output;
    double Lvoltage = output;
    Rvoltage = Utility.Constrain(Rvoltage, -Constants.chassisVisionPIDMaxOutputVoltage, Constants.chassisVisionPIDMaxOutputVoltage);
    Lvoltage = Utility.Constrain(Lvoltage, -Constants.chassisVisionPIDMaxOutputVoltage, Constants.chassisVisionPIDMaxOutputVoltage);
    setMotorVoltage(Lvoltage, Rvoltage);
  }
  public boolean visionPIDIsStable(){
    return visionPID.atSetpoint();
  }


  //Left motor PID function
  public void LMotorPIDEnable(){
    LMotorPIDEnable = true;
    LMotorPID.reset();
  }
  public void LMotorPIDDisable(){
    LMotorPIDEnable = false;
    LMotorPIDOutput(0);
  }
  public void LMotorPIDReset(){
    LMotorPID.reset();
  }
  public boolean LMotorPIDIsEnable(){
    return LMotorPIDEnable;
  }
  public void LMotorPIDSetSetpoint(double setpoint){
    LMotorPIDSetpoint = setpoint;
  }
  public void LMotorPIDSetTolerance(double value){
    LMotorPID.setTolerance(value);
  }
  public double LMotorPIDMeasurment(){
    if(Constants.chassisMotorLEncoderInverted){
      return (-1.0 * getLeftMotorEncoder());
    }else{
      return getLeftMotorEncoder();
    }
  }
  public void LMotorPIDOutput(double output){
    double Lvoltage = output;
    Utility.Constrain(Lvoltage, -Constants.chassisMotorPIDMaxOutputVoltage, Constants.chassisMotorPIDMaxOutputVoltage);
    setLeftMotorVoltage(Lvoltage); 
  }
  public boolean LMotorPIDIsStable(){
    return LMotorPID.atSetpoint();
  }

  //Right motor PID function
  public void RMotorPIDEnable(){
    RMotorPIDEnable = true;
    RMotorPID.reset();
  }
  public void RMotorPIDDisable(){
    RMotorPIDEnable = false;
    RMotorPIDOutput(0);
  }
  public void RMotorPIDReset(){
    RMotorPID.reset();
  }
  public boolean RMotorPIDIsEnable(){
    return RMotorPIDEnable;
  }
  public void RMotorPIDSetSetpoint(double setpoint){
    RMotorPIDSetpoint = setpoint;
  }
  public void RMotorPIDSetTolerance(double value){
    RMotorPID.setTolerance(value);
  }
  public double RMotorPIDMeasurment(){
    if(Constants.chassisMotorREncoderInverted){
      return (-1.0 * getRightMotorEncoder());
    }else{
      return getRightMotorEncoder();
    }
  }
  public void RMotorPIDOutput(double output){
    double Rvoltage = output;
    Utility.Constrain(Rvoltage, -Constants.chassisMotorPIDMaxOutputVoltage, Constants.chassisMotorPIDMaxOutputVoltage);
    setRightMotorVoltage(Rvoltage); 
  }
  public boolean RMotorPIDIsStable(){
    return RMotorPID.atSetpoint();
  }
}
