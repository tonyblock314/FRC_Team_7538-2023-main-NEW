// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Experimentation file for command based

package frc.robot.commands.Elevator;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class UpAndDown extends CommandBase {
  double setpoint = 0;
  double lastUpdate = Timer.getFPGATimestamp();
  private boolean down, up;
  private Elevator m_subsystem;
  RelativeEncoder encoder;

  /** Creates a new UpAndDown. */
  public UpAndDown(Elevator subsystem, boolean down, boolean up) {
    m_subsystem = subsystem;
    this.down = down;
    this.up = up;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 30;
    double out = 0;
    if(down) out-=speed;
    if(up) out+=speed;
    //double feedForward = out/20.0;
    double position = encoder.getPosition();
    
    setpoint+= out*(Timer.getFPGATimestamp() - lastUpdate);
    lastUpdate = Timer.getFPGATimestamp();

    double pLoopOut = (setpoint - position)/10.0;


    m_subsystem.setSpark(pLoopOut);
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
