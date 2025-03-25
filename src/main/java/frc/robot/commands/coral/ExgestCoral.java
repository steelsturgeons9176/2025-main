package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.EndifactorSubsystem;

public class ExgestCoral extends Command{
    private EndifactorSubsystem m_endifactor;


    public ExgestCoral(EndifactorSubsystem endifactor){
        m_endifactor = endifactor;
        addRequirements(m_endifactor);
    }

    @Override
    public void initialize(){
        m_endifactor.setCoralVoltage(4);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_endifactor.setCoralVoltage(0);
    }
}
