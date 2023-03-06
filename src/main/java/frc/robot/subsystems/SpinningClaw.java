package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpinningClaw extends SubsystemBase {
    TalonFX talon = new TalonFX(Constants.CLAW_CAN_ID);

    public SpinningClaw() {
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSelectedSensorPosition(0);
    }
    
    double setpoint = 0;
    double lastUpdate = Timer.getFPGATimestamp();
    
    public void periodic(double rightstickY) {
        if(Math.abs(rightstickY)<Constants.LT_DEADBAND) rightstickY = 0;
        
        double out = (rightstickY) * Constants.SCALING_FACTOR;
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
}
