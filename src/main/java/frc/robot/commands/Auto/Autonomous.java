// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveTrain.setMotorMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PistonClaw;
import frc.robot.subsystems.DriveTrain.Mode;


public class Autonomous extends SequentialCommandGroup {

  /***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
  AHRS navx = new AHRS(SPI.Port.kMXP); 

  double rolldegrees = navx.getRoll();
  double tiltdegrees = navx.getYaw();
  double pitchdegrees = navx.getPitch();

  /**
   * First, gets drive train to drive backwards for a practically determined amount of time;
   * Finally, turns all off
   */
  // 0.305 * 5.2 = 5.5 ft

  public Autonomous(DriveTrain driveTrain, PistonClaw claw) {
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
      new AutoDrive(driveTrain, 0.305 * 1),
      new Wait(driveTrain, 3000),
      new AutoDrive(driveTrain, 0.305 * -10.4), //Drive 11 feet back
      new Wait(driveTrain, 1000),
      new setMotorMode(driveTrain, Mode.COAST));

    */

    // Tried and True Autonomus
    /* 
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new Wait(driveTrain, 3000),
      new AutoDrive(driveTrain, 0.305 * -10.4), //Drive 11 feet back
      new Wait(driveTrain, 500),
      new setMotorMode(driveTrain, Mode.COAST));
    */
    
    //Auto Balancing Autonomous (please work)
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new Wait(driveTrain, 2000),
      new AutoDrive(driveTrain, 0.305 * -10.4),
      new AutoDrive(driveTrain, 0.305 * 2));
      if (pitchdegrees <= Constants.TILT_THRESHOLD_BACKWARDS) {
        addCommands(new AutoDrive(driveTrain, 0.1));
      } else if(pitchdegrees >= Constants.TILT_THRESHOLD_FORWARDS) {
        addCommands(new AutoDrive(driveTrain, -0.1));
      } else {
        addCommands(new Wait(driveTrain, 50));
      }
  }
}