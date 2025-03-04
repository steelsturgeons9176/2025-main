package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class WristScoreMid extends Command{
    private Intake m_intake;


    public WristScoreMid(Intake intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_intake.wristAngle(Intake.wristPositions.L2_ANGLE);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_intake.wristAngle(Intake.wristPositions.HOLD_ANGLE);
    }
}
