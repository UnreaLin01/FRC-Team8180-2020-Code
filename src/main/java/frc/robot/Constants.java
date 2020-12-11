/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2049 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {
    //Joystick index defined
    public static final byte buttonY = 4;
    public static final byte buttonA = 1;
    public static final byte buttonX = 3;
    public static final byte buttonB = 2;
    public static final byte buttonRB = 6;
    public static final byte buttonLB = 5;
    public static final byte buttonBack = 7;
    public static final byte buttonOption = 8;
    public static final byte buttonJR = 10;
    public static final byte buttonJL = 9;
    
    public static final byte axisJRX = 4;
    public static final byte axisJRY = 5;
    public static final byte axisJLX = 0;
    public static final byte axisJLY = 1;
    public static final byte axisRT = 3;
    public static final byte axisLT = 2;

    //Joystick Setting
    public static final boolean axisRXInvert = false;
    public static final boolean axisRYInvert = true;
    public static final boolean axisLXInvert = false;
    public static final boolean axisLYInvert = true;

    public static final double joystickDeadZone = 0.2;
    public static final double axisRXExp = 1;
    public static final double axisRXScale = 1;
    public static final double axisRYExp = 1;
    public static final double axisRYScale = 1;
    public static final double axisLXExp = 1;
    public static final double axisLXScale = 1;
    public static final double axisLYExp = 1;
    public static final double axisLYScale = 1;

    


    
    //Chassis Setting
    public static final byte chassisMotorRFID = 1; //ID
    public static final byte chassisMotorRBID = 2;  //ID
    public static final byte chassisMotorLFID = 3; //ID
    public static final byte chassisMotorLBID = 4; //ID

    public static final boolean chassisMotorRInverted = false;
    public static final boolean chassisMotorLInverted = false;
    public static final boolean chassisMotorREncoderInverted = false;
    public static final boolean chassisMotorLEncoderInverted = false;

    public static final double chassisMotorNormalModeSpeedScale = 0.5;
    public static final double chassisMotorSlowModeSpeedScale = 0.3;

    public static final double chassisWheelPerimeter = 6 * 0.0254 * Math.PI;

    public static final double chassisHeadingPIDKp = 0.1;
    public static final double chassisHeadingPIDKi = 0;
    public static final double chassisHeadingPIDKd = 0.01;
    public static final double chassisHeadingPIDRestartTime = 0.25;

    public static final double chassisTurnPIDKp = 0.1;
    public static final double chassisTurnPIDKi = 0;
    public static final double chassisTurnPIDKd = 0.01;
    public static final double chassisTurnPIDMaxOutputVoltage = 5;

    public static final double chassisVisionPIDKp = 0.025;
    public static final double chassisVisionPIDKi = 0;
    public static final double chassisVisionPIDKd = 0.01;
    public static final double chassisVisionPIDMaxOutputVoltage = 5;

    public static final double chassisMotorPIDKp = 0.5;
    public static final double chassisMotorPIDKi = 0;
    public static final double chassisMotorPIDKd = 0;
    public static final double chassisMotorPIDMaxOutputVoltage = 5;





    //Shooter Setting
    public static final byte shooterContainerMotorID = 51;  //ID
    public static final byte shooterFeedMotorID = 21;       //ID
    public static final byte shooterUpperMotorID = 22;      //ID
    public static final byte shooterLowerMotorID = 23;      //ID
    public static final boolean shooterContainerMotorInverted = false;
    public static final boolean shooterFeedMotorInverted = false;
    public static final boolean shooterUpperMotorInverted = true;
    public static final boolean shooterLowerMotorInverted = false;

    public static final double shooterUpperMotorMaxPIDRPS = 60;
    public static final double shooterLowerMotorMaxPIDRPS = 60;
    public static final int shooterUpperEncoderPPR = 2048;
    public static final int shooterLowerEncoderPPR = 2048;
    public static final byte shooterUpperEncoderPinA = 7;
    public static final byte shooterUpperEncoderPinB = 6;
    public static final boolean shooterUpperEncoderDirectionInverted = true;
    public static final byte shooterLowerEncoderPinA = 9;
    public static final byte shooterLowerEncoderPinB = 8;
    public static final boolean shooterLowerEncoderDirectionInverted = false;
    
    



    //Climber Setting
    public static final byte climberLiftMotorID = 11;       //ID
    public static final byte climberHookFrontMotorID = 12;  //ID
    public static final byte climberHookBackMotorID = 13;   //ID
    public static final boolean climberHookMotorInverted = false;
    public static final boolean climberLiftMotorInverted = false;

    public static final byte climberLiftTopLimitSwID = 0;
    public static final byte climberLiftBottomLimitSwID = 0;
    




    //Intake Setting
    public static final byte intakeLiftMotorID = 31; //ID
    public static final byte intakeSpinMotorID = 32; //ID
    public static final boolean intakeLiftMotorInverted = false;
    public static final boolean intakeSpinMotorInverted = false;

    public static final double intakeLiftMotorGearRatio = 188;
    public static final byte intakeLiftEncoderPinA = 0;
    public static final byte intakeLiftEncoderPinB = 0;
    public static final boolean intakeLiftEncoderInverted = false;
    
    public static final double intakeLiftPIDKp = 0;
    public static final double intakeLiftPIDKi = 0;
    public static final double intakeLiftPIDKd = 0;
    public static final double intakeLiftPIDMaxOutputVoltage = 12;

    public static final byte intakeZeroLimitSwID = 0;





    //Spinner Setting
    public static final byte spinnerLiftMotorID = 41; //ID
    public static final byte spinnerSpinMotorID = 42; //ID
    public static final boolean spinnerLiftMotorInverted = false;
    public static final boolean spinnerSpinMotorInverted = false;

    public static final double spinnerLiftMotorGearRatio = 188;
    public static final byte spinnerLiftEncoderPinA = 0;
    public static final byte spinnerLiftEncoderPinB = 0;
    public static final boolean spinnerLiftEncoderInverted = false;

    public static final double spinnerLiftPIDKp = 0;
    public static final double spinnerLiftPIDKi = 0;
    public static final double spinnerLiftPIDKd = 0;
    public static final double spinnerLiftPIDMaxOutputVoltage = 12;

    public static final byte spinnerZeroLimitSwID = 0;

}
