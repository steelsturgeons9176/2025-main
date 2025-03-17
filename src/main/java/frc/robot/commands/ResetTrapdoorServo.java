package frc.robot.commands;


import edu.wpi.first.wpilibj.Servo; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapDoorSubsystem;

public class ResetTrapdoorServo extends Command{
    private TrapDoorSubsystem m_trapdoor;

    public ResetTrapdoorServo(TrapDoorSubsystem servo){
        m_trapdoor = servo;
        addRequirements(m_trapdoor);
    }
    


    @Override
    public void initialize(){
        m_trapdoor.setServoAngle(0);
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
