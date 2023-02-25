package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PistonClaw;

public class CloseClaw extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final PistonClaw m_claw;

    public CloseClaw(PistonClaw pistonclawsubsystem) {
        m_claw = pistonclawsubsystem;
        addRequirements(pistonclawsubsystem);
    }
    
    public void execute() {
        m_claw.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
