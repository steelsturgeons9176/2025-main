package frc.robot.commands.funnel.trapdoor;


import edu.wpi.first.wpilibj.Servo; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapDoorSubsystem;

public class ActivateTrapdoor extends Command{
    private TrapDoorSubsystem m_trapdoor;

    public ActivateTrapdoor(TrapDoorSubsystem servo){
        m_trapdoor = servo;
        addRequirements(m_trapdoor);
    }
    


    @Override
    public void initialize(){
        m_trapdoor.setServoAngle(70);
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

    }
}
