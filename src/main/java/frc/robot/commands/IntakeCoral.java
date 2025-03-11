package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndifactorSubsystem;


public class IntakeCoral extends Command{
    private EndifactorSubsystem m_endifactor;
    private boolean isSeen;


    public IntakeCoral(EndifactorSubsystem endifactor){
        m_endifactor = endifactor;
        addRequirements(m_endifactor);
    }

    @Override
    public void initialize(){
        m_endifactor.setCoralVoltage(12);
        isSeen = false;
    }

    @Override
    public void execute(){
        if (isSeen == false && m_endifactor.hasCoral() == true) {
            isSeen = true;
            m_endifactor.setCoralVoltage(8);
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
        m_endifactor.setCoralVoltage(0);
        isSeen = false;
    }
}
