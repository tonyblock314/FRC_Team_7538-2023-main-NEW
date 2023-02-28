package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PistonClaw extends SubsystemBase{
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private final DoubleSolenoid claw;

    boolean pressureSwitch;

    public PistonClaw() {
        claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        
        //pressureSwitch = pcmCompressor.getPressureSwitchValue();
        //if (pressureSwitch) {
            //pcmCompressor.disable();
        //}
        //else {
            //pcmCompressor.enableDigital();
        //}

    }
        
    public void retract() {
        claw.set(DoubleSolenoid.Value.kReverse);
        //new WaitCommand(1);
        //claw.set(DoubleSolenoid.Value.kOff);
    }
        
    public void extend() {
        claw.set(DoubleSolenoid.Value.kForward);
        //new WaitCommand(1);
        //claw.set(DoubleSolenoid.Value.kOff);
    }
    
}
