package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PistonClaw;

public class OpenClaw extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final PistonClaw m_claw;

    public OpenClaw(PistonClaw pistonclawsubsystem) {
        m_claw = pistonclawsubsystem;
        addRequirements(pistonclawsubsystem);
    }
    
    public void execute() {
        m_claw.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
