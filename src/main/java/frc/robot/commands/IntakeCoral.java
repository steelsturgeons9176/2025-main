package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndifactorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;


public class IntakeCoral extends Command{
    private EndifactorSubsystem m_endifactor;
    private FunnelSubsystem m_funnel;
    private boolean isSeen;
    public IntakeCoral(EndifactorSubsystem endifactor, FunnelSubsystem funnel){
        m_endifactor = endifactor;
        m_funnel = funnel;
        addRequirements(m_funnel);
        addRequirements(m_endifactor);    }

    @Override
    public void initialize(){
        m_endifactor.setCoralDutyCylce(.15);
        isSeen = false;
        m_funnel.setVoltage(12);
    }

    @Override
    public void execute(){
        if (isSeen == false && m_endifactor.hasCoral() == true) {
            isSeen = true;
         //   m_endifactor.setCoralVoltage(8);
         m_endifactor.setCoralDutyCylce(.15);
        }
        else if(isSeen == true && m_endifactor.hasCoral() == false){
            end(true);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        isSeen = false;
        m_funnel.setVoltage(0);
        m_endifactor.setPosition(m_endifactor.getPosition());
    }
}
