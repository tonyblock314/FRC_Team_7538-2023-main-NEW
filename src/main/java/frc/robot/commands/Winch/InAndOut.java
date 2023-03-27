// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Experimentation file for command based

package frc.robot.commands.Winch;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Winch;

public class InAndOut extends CommandBase {
  double setpoint = 0;
  double lastUpdate = Timer.getFPGATimestamp();
  private Winch m_subsystem;
  private DoubleSupplier leftdepressed, rightdepressed;

  /** Creates a new InAndOut. */
  public InAndOut(Winch subsystem, DoubleSupplier leftdepressed, DoubleSupplier rightdepressed) {
    m_subsystem = subsystem;
    this.leftdepressed = leftdepressed;
    this.rightdepressed = rightdepressed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

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

    double out = (right-left) * 4;
    double feedForward = out/20.0;
    double position = m_subsystem.getEncoderPosition();
    
    setpoint+= out*(Timer.getFPGATimestamp() - lastUpdate);



    lastUpdate = Timer.getFPGATimestamp();

    double pLoopOut = (setpoint - position)* 0.2;
    // if(pLoopOut>0.2) pLoopOut = 0.2;

    SmartDashboard.putNumber("Winch Setpoint", setpoint);
    SmartDashboard.putNumber("Winch pLoopOut", pLoopOut);
    m_subsystem.setWinchMotor(pLoopOut + feedForward);
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
