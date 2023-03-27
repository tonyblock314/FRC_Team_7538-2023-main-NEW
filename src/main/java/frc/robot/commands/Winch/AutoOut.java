// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Winch;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class AutoOut extends CommandBase {
  /*
  double setpoint = 0;
  double lastUpdate = Timer.getFPGATimestamp();
  */
  private Winch m_subsystem;
  private double input;
  /** Creates a new AutoOut. */
  public AutoOut(Winch subsystem, double input) {
    m_subsystem = subsystem;
    this.input = input;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    double out = input;
    double feedForward = out/20.0;
    double position = m_subsystem.getEncoderPosition();
    
    setpoint+= out*(Timer.getFPGATimestamp() - lastUpdate);
    lastUpdate = Timer.getFPGATimestamp();

    double pLoopOut = (setpoint - position)* 0.2;
    // if(pLoopOut>0.2) pLoopOut = 0.2;
    */
    m_subsystem.setWinchMotor(input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
