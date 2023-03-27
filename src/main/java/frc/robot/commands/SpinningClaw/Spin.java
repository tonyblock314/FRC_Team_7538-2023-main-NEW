// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpinningClaw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SpinningClaw;

public class Spin extends CommandBase {
  private SpinningClaw m_subsystem;
  private DoubleSupplier leftdepressed, rightdepressed;
  
  /** Creates a new Spin. */
  public Spin(SpinningClaw subsystem, DoubleSupplier leftdepressed, DoubleSupplier rightdepressed) {
    this.m_subsystem = subsystem;
    this.leftdepressed = leftdepressed;
    this.rightdepressed = rightdepressed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left, right;
    if(leftdepressed.getAsDouble()<Constants.LT_DEADBAND) {
      left = 0;
    } else {
      left = leftdepressed.getAsDouble();
    }
    if(rightdepressed.getAsDouble()<Constants.RT_DEADBAND) {
      right = 0;
    } else {
      right = rightdepressed.getAsDouble();
    }

    double speed = (left - right) * Constants.SCALING_FACTOR_CLAW;
    m_subsystem.setClawSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
