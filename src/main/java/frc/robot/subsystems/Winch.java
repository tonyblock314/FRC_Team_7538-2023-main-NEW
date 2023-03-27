// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/** Add your docs here. */
public class Winch extends SubsystemBase {
  double setpoint = 0;
  double lastUpdate = Timer.getFPGATimestamp();
  TalonFX talon = new TalonFX(Constants.TOP_WINCH_1_CAN_ID);

  public Winch() {
    talon.configFactoryDefault();
    talon.setNeutralMode(NeutralMode.Brake);
    talon.setSelectedSensorPosition(0);
  }

  public double getEncoderPosition() {
    return talon.getSelectedSensorPosition()/8192;
  }

  public void setWinchMotor(double speed) {
    talon.set(ControlMode.PercentOutput, speed);
  }

  /*
  public void periodic(double left, double right){
    if(left<Constants.LT_DEADBAND) left = 0;
    if(right<Constants.RT_DEADBAND) right = 0;
    double out = (right-left) * 3.5;
    double feedForward = out/20.0;
    double position = talon.getSelectedSensorPosition()/8192.0;
    
    setpoint+= out*(Timer.getFPGATimestamp() - lastUpdate);
    lastUpdate = Timer.getFPGATimestamp();

    double pLoopOut = (setpoint - position)* 0.2;
  //  if(pLoopOut>0.2) pLoopOut = 0.2;

    talon.set(ControlMode.PercentOutput,pLoopOut+feedForward);
    SmartDashboard.putNumber("winch_pLoopOut", pLoopOut);
    SmartDashboard.putNumber("winchSetpoint", setpoint);
    SmartDashboard.putNumber("winchEncoderReading",talon.getSelectedSensorPosition()/8192.0);
    SmartDashboard.putNumber("winchSpeed", out);
  }
  */
}
