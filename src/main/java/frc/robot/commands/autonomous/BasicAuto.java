/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.DriveAngle;
import frc.robot.commands.chassis.DriveDistance;
import frc.robot.commands.chassis.DriveTime;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class BasicAuto extends SequentialCommandGroup {
  public BasicAuto(Chassis m_chassis) {
    super(
      new DriveTime(m_chassis, 2.0, 0.0, 0.0), //Set Motor Left Speed 0.2, Right Speed 0.2 and Drive 1 Second
      new DriveTime(m_chassis, 2.0, 1.0, -1.0),
      new DriveTime(m_chassis, 2.0, -1.0, -1.0),
      new DriveTime(m_chassis, 2.0, -1.0, 1.0)
      //new DriveDistance(m_chassis,1,1), //go straight 1m
      //new DriveAngle(m_chassis, 90),    //turn clockwise 90deg
      //new DriveDistance(m_chassis,1,1), //go straight 1m
      //new DriveAngle(m_chassis, 90),    //turn clockwise 90deg
      //new DriveDistance(m_chassis,1,1), //go straight 1m
      //new DriveAngle(m_chassis, 90),    //turn clockwise 90deg
      //new DriveDistance(m_chassis,1,1), //go straight 1m
      //new DriveAngle(m_chassis, 90)     //turn clockwise 90deg
    );
  }
}
