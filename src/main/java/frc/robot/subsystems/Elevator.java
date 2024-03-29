// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/** Add your docs here. */
public class Elevator extends SubsystemBase {

  //WPI_VictorSPX Winch = new WPI_VictorSPX(Constants.TOP_WINCH_1_CAN_ID);
  //TalonFX talon = new TalonFX(Constants.TOP_WINCH_1_CAN_ID);
  CANSparkMax spark = new CANSparkMax(Constants.Leader_SparkMax_ID,MotorType.kBrushless);
  CANSparkMax sparkFollower = new CANSparkMax(Constants.Follower_SparkMax_ID,MotorType.kBrushless);
  RelativeEncoder encoder;


  public Elevator() {
    //talon.configFactoryDefault();
    //talon.setNeutralMode(NeutralMode.Brake);
    //talon.setInverted(true);
    //talon.setSelectedSensorPosition(0);

    spark.restoreFactoryDefaults();
    sparkFollower.restoreFactoryDefaults();
    spark.setSmartCurrentLimit(30);
    sparkFollower.setSmartCurrentLimit(30);
    sparkFollower.follow(spark,true);
    spark.setIdleMode(IdleMode.kBrake);
    sparkFollower.setIdleMode(IdleMode.kBrake);
    encoder = spark.getEncoder();
    encoder.setPosition(0);

    
   // spark.sensor
    
  }
double setpoint = 0;
double lastUpdate = Timer.getFPGATimestamp();

  public void periodic(boolean down, boolean up){
    double speed = 8;
    double out = 0;
    if(down) out-=speed;
    if(up) out+=speed;
    double feedForward = out/20.0;
    double position = encoder.getPosition();
    
    setpoint+= out*(Timer.getFPGATimestamp() - lastUpdate);
    lastUpdate = Timer.getFPGATimestamp();

    double pLoopOut = (setpoint - position)/10.0;
    if(pLoopOut>0.2) pLoopOut = 0.2;

    spark.set(pLoopOut);
    SmartDashboard.putNumber("encoderSetpoint", setpoint);
    SmartDashboard.putNumber("sparkEncoderReading",encoder.getPosition());
    SmartDashboard.putNumber("elevatorPLoopOut", pLoopOut);
    SmartDashboard.putNumber("feedforward", feedForward);
  }
   // Pushes new speed to intake wheel motor

}
