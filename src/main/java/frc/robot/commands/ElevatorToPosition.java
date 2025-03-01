package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToPosition extends Command {
    private ElevatorSubsystem m_elevator;
    private ElevatorSubsystem.elevatorPositions m_tarPosition;
    

    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.elevatorPositions pos){
        m_elevator = elevator;
        m_tarPosition = pos;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_elevator.raiseArmAbs(m_tarPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        
    }
}
