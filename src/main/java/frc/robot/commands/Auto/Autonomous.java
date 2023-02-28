// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.DriveTrain.setMotorMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PistonClaw;
import frc.robot.subsystems.DriveTrain.Mode;


public class Autonomous extends SequentialCommandGroup {
  /**
   * First, gets drive train to drive backwards for a practically determined amount of time;
   * Finally, turns all off
   */
  // 0.305 * 5.2 = 5.5 ft
  private final PistonClaw m_claw = new PistonClaw();

  public Autonomous(DriveTrain driveTrain) {
    /** 
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new AutoDrive(driveTrain, 0.305 * 5.2), // Drive 5.5 feet
      new Wait(driveTrain, 1000),
      new setMotorMode(driveTrain, Mode.COAST));
    */
    /** 
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new OpenClaw(m_claw),
      new Wait(driveTrain, 1000),
      new AutoDrive(driveTrain, 0.305 * -15.6), //Drive 16.5 feet back
      new Wait(driveTrain, 1000),
      new setMotorMode(driveTrain, Mode.COAST));
    */
    
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new OpenClaw(m_claw),
      new Wait(driveTrain, 1000),
      new AutoDrive(driveTrain, 0.305 * -12), //Drive 12.69 feet back
      new Wait(driveTrain, 500),
      new setMotorMode(driveTrain, Mode.COAST));
    
  }
  
}