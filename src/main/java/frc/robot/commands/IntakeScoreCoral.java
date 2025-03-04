package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeScoreCoral extends Command{
    private Intake m_intake;


    public IntakeScoreCoral(Intake intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_intake.setCoralIntakeVoltage(-4);
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
        m_intake.setCoralIntakeVoltage(0);
    }
}
