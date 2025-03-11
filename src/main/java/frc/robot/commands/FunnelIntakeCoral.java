package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;

public class FunnelIntakeCoral extends Command{
    private FunnelSubsystem m_funnel;


    public FunnelIntakeCoral(FunnelSubsystem funnel){
        m_funnel = funnel;
        addRequirements(m_funnel);
    }

    @Override
    public void initialize(){
        m_funnel.setVoltage(5);
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
        m_funnel.setVoltage(0);
    }
}
