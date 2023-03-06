// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.cameraserver.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CANSparkMax m_motor;
  private CANSparkMax m_motor2;
  private static final int deviceID = 1;
  private static final int deviceID2 = 2;
  private RobotContainer m_robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(deviceID2, MotorType.kBrushless);
    CameraServer.startAutomaticCapture();

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);


    m_motor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_motor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_motor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    m_motor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);


    SmartDashboard.putNumber("Forward Soft Limit", m_motor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Reverse Soft Limit", m_motor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Gets the autonomous command as assigned in the robot container
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_motor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_motor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    m_motor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    m_motor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
