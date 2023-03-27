// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveTrain.setMotorMode;
import frc.robot.commands.SpinningClaw.Shoot;
import frc.robot.commands.Winch.AutoOut;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SpinningClaw;
import frc.robot.subsystems.Winch;
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

  public Autonomous(DriveTrain driveTrain, SpinningClaw claw, Winch winch) {
    /*
    addCommands(
      new Wait(claw, 1000),
      new Shoot(claw, -0.25),
      new Wait(claw, 500),
      new Shoot(claw, 0),
      new Wait(claw, 1000)
    );
    */
    /**
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new Wait(driveTrain, 1000),
      new AutoDrive(driveTrain, 0.305 * -10.4),
      new Wait(driveTrain, 1000),
      new AutoDrive(driveTrain, 0.305 * 10.4),
      new setMotorMode(driveTrain, Mode.COAST));

    */

    // Tried and True Autonomus
     /* 
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new AutoDrive(driveTrain, -0.5),
      new Wait(driveTrain, 100),
      new AutoDriveSlow(driveTrain, 0.5),
      new Wait(winch, 750),
      new AutoOut(winch, -0.1),
      new Wait(winch, 500),
      new AutoOut(winch, 0),
      new Wait(claw, 1000),
      new Shoot(claw, -0.5),
      new Wait(claw, 500),
      new Shoot(claw, 0),
      new Wait(claw, 3000),
      new AutoDrive(driveTrain, -3.172), //Drive 11 feet back
      new Wait(driveTrain, 500),
      new setMotorMode(driveTrain, Mode.COAST));
     */
    /* 
    // Some test auto
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new AutoDrive(driveTrain, -0.5),
      new Wait(driveTrain, 100),
      new AutoDriveSlow(driveTrain, 0.5),
      new Wait(winch, 500),
      new AutoOut(winch, -0.1),
      new Wait(winch, 500),
      new Shoot(claw, -0.5),
      new Wait(claw, 500),
      new Shoot(claw, 0),
      new AutoOut(winch,0.1),
      new Wait(winch, 500),
      new AutoDrive(driveTrain, 3.172),
      new Wait(driveTrain, 100),
      new AutoManualTurn(driveTrain, 0, -0.4),
      new Wait(driveTrain, 300),
      new AutoOut(winch, -1.2),
      new Shoot(claw, 0.8),
      new Wait(claw, 500),
      new AutoDriveSlow(driveTrain, 0.5),
      new Shoot(claw, 0),
      new AutoOut(winch, 1.2),
      new AutoDrive(driveTrain, -0.5),
      new Wait(driveTrain, 300),
      new AutoManualTurn(driveTrain, 0, -0.4),
      new AutoDrive(driveTrain, 3.172),
      new AutoOut(winch, -0.1),
      new Shoot(claw, -0.35),
      new Shoot(claw, 0),
      new AutoOut(winch, 0.1)
      ); */


    //Auto Balancing Autonomous (please work)
    /* 
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      
      new Wait(winch, 500),
      new AutoOut(winch, -0.1),
      new Wait(winch, 500),
      new AutoOut(winch, 0),
      
      new Wait(claw, 1000),
      new Shoot(claw, -0.25),
      new Wait(claw, 300),
      new Shoot(claw, 0),
      new Wait(driveTrain, 1000),
      new AutoDrive(driveTrain, 0.305 * -10.4),
      new Wait(driveTrain, 500),
      new AutoDrive(driveTrain, 0.305 * 2));

      SmartDashboard.putNumber("pitch1", pitchdegrees);
      SmartDashboard.putNumber("roll1", rolldegrees);
      SmartDashboard.putNumber("yaw1", tiltdegrees);

    if (pitchdegrees <= Constants.TILT_THRESHOLD_BACKWARDS) {
      SmartDashboard.putNumber("pitch2", pitchdegrees);
      SmartDashboard.putNumber("roll2", rolldegrees);
      SmartDashboard.putNumber("yaw2", tiltdegrees);
      addCommands(new AutoDriveSlow(driveTrain, 0.1));
    } else if(pitchdegrees >= Constants.TILT_THRESHOLD_FORWARDS) {
      SmartDashboard.putNumber("pitch3", pitchdegrees);
      SmartDashboard.putNumber("roll3", rolldegrees);
      SmartDashboard.putNumber("yaw3", tiltdegrees);
      addCommands(new AutoDriveSlow(driveTrain, -0.1));
    } else {
      SmartDashboard.putNumber("pitch4", pitchdegrees);
      SmartDashboard.putNumber("roll4", rolldegrees);
      SmartDashboard.putNumber("yaw4", tiltdegrees);
      addCommands(new Wait(driveTrain, 50));
    }
    */
    //Auto Balancing Autonomous (work!)
    
    addCommands(
      new setMotorMode(driveTrain, Mode.BRAKE),
      new Wait(driveTrain, 1000),
      new AutoDriveSlow(driveTrain, 1.82),
      new Wait(driveTrain, 500));

      SmartDashboard.putNumber("pitch1", pitchdegrees);
      SmartDashboard.putNumber("roll1", rolldegrees);
      SmartDashboard.putNumber("yaw1", tiltdegrees);

    if (pitchdegrees <= Constants.TILT_THRESHOLD_BACKWARDS) {
      SmartDashboard.putNumber("pitch2", pitchdegrees);
      SmartDashboard.putNumber("roll2", rolldegrees);
      SmartDashboard.putNumber("yaw2", tiltdegrees);
      addCommands(new AutoDriveSlow(driveTrain, 0.1));
    } else if(pitchdegrees >= Constants.TILT_THRESHOLD_FORWARDS) {
      SmartDashboard.putNumber("pitch3", pitchdegrees);
      SmartDashboard.putNumber("roll3", rolldegrees);
      SmartDashboard.putNumber("yaw3", tiltdegrees);
      addCommands(new AutoDriveSlow(driveTrain, -0.1));
    } else {
      SmartDashboard.putNumber("pitch4", pitchdegrees);
      SmartDashboard.putNumber("roll4", rolldegrees);
      SmartDashboard.putNumber("yaw4", tiltdegrees);
      addCommands(new Wait(driveTrain, 50));
    } 
  }
}